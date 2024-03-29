#include <common.h>

using namespace gr_control_gui;

MyCommonViz::MyCommonViz( QWidget* parent): QWidget( parent ), nh_{},  robot_radius_(1.5),
                 nrows_{1}, y_cells_{1}, terrain_x_(1.0), terrain_y_(1.0), id_maxnumberrows_(1),
                 angle_{0.0}, direction_{-1}, default_npoints_{3}, mongo_connection_(){
  mongo_connection_.connect();
  ROS_INFO_STREAM("Connected "<< mongo_connection_.isConnected());
  map_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("full_topological_map", 1 );
  region_publisher_ = nh_.advertise<visualization_msgs::Marker>("region", 1 );

  main_layout_ = new QVBoxLayout();
  render_panel_ = new rviz::RenderPanel();
  controls_layout_ = new QGridLayout();
  QPushButton* load_topological_map = new QPushButton ("Load Map");
  controls_layout_->addWidget( load_topological_map, 0, 0 );
  connect( load_topological_map, SIGNAL( released( )), this, SLOT( loadMap()));

  manager_ = new rviz::VisualizationManager( render_panel_ );

  return;

  manager_->setFixedFrame(QString("map"));

  
  auto controller = manager_->getViewManager()->create(QString("rviz/ThirdPersonFollower"));
  render_panel_->initialize( manager_->getSceneManager(), manager_ );

  // Create Subscribers
  rviz::Display* marker_display;
  marker_display = manager_->createDisplay( "rviz/MarkerArray", "topological_map", true );
  ROS_ASSERT( marker_display != NULL );
  marker_display->subProp( "Marker Topic" )->setValue("full_topological_map");

  rviz::Display* region_marker;
  region_marker = manager_->createDisplay( "rviz/Marker", "region_marker", true );
  ROS_ASSERT( region_marker != NULL );
  region_marker->subProp( "Marker Topic" )->setValue("region");

  rviz::Display* proximity_marker;
  proximity_marker = manager_->createDisplay( "rviz/MarkerArray", "proximity_marker", true );
  ROS_ASSERT( proximity_marker != NULL );
  proximity_marker->subProp( "Marker Topic" )->setValue("proximity_visualization");

  rviz::Display* robot_display;
  robot_display = manager_->createDisplay( "rviz/RobotModel", "thorvald", true );
  ROS_ASSERT( robot_display != NULL );
  //robot_display->subProp( "Marker Topic" )->setValue("region");

  rviz::Display* robot_path;
  robot_path = manager_->createDisplay( "rviz/Path", "robot_path", true );
  ROS_ASSERT( robot_path != NULL );
  robot_path->subProp( "Topic" )->setValue("gr_sbpl_trajectory_generator_node/plan");
  //robot_path->subProp( "Pose Style" )->setValue(2);

  rviz::Display* map;
  map = manager_->createDisplay( "rviz/Map", "safety_map", true );
  ROS_ASSERT( map != NULL );
  map->subProp( "Topic" )->setValue("/grid_map_visualization/safety_costmap");

  rviz::Display* gmap;
  gmap = manager_->createDisplay( "rviz/Map", "main_map", true );
  ROS_ASSERT( gmap != NULL );
  gmap->subProp( "Topic" )->setValue("/map");

  rviz::Display* pc;
  pc = manager_->createDisplay( "rviz/PointCloud2", "velodyne_pc", true );
  ROS_ASSERT( pc != NULL );
  pc->subProp( "Topic" )->setValue("/velodyne_points");

  rviz::Display* dect;
  dect = manager_->createDisplay( "jsk_rviz_plugin/BoundingBoxArray", "dect_bb", true );
  ROS_ASSERT( dect != NULL );
  dect->subProp( "Topic" )->setValue("/detection/bounding_boxes");
  ROS_INFO("Ok");

}

void MyCommonViz::loadGUI(){
	manager_->initialize();
	manager_->setFixedFrame(QString("map"));
	manager_->startUpdate();
	//manager_->getViewManager()->setCurrentViewControllerType("rviz/ThirdPersonFollower");
	main_layout_->addLayout( controls_layout_ );
	
	//main_layout_->addWidget( render_panel_ );
	setLayout( main_layout_ );
}

void MyCommonViz::loadMap(){
  TopoMapCollection mongo_coll = mongo_connection_.openCollection<navigation_msgs::TopologicalMap>("my_db", "maps");
  warehouse_ros::Query::Ptr q1 = mongo_coll.createQuery();
  q1->append("name", "wish_map4");
  std::vector<TopoMapMetaPtr> res = mongo_coll.queryList(q1, true);
  ROS_INFO_STREAM("SIZE " << res.size());
  parseMessage(load_map_, res[0]);
  ROS_INFO_STREAM(load_map_);

	ROS_WARN_STREAM(load_map_.info);

	angle_ = load_map_.info.angle_offset;
	direction_ = load_map_.info.direction;
	map_frame_ = load_map_.info.map_frame;
	robot_radius_ = load_map_.info.robot_radius;
	terrain_y_ = load_map_.info.sizey;
	terrain_x_ = load_map_.info.sizex;
	nrows_ = ceil(terrain_x_/(2*robot_radius_));
	std::cout << "MAX XCELLS " <<  nrows_ << std::endl;
	y_cells_ =  default_npoints_;//ceil(terrain_y_/1);
	id_maxnumberrows_ = nrows_-1;

	visualizeMap();
	updateAfterLoad();
	gr_map_utils::UpdateMap req;
	if(map_client_.call(req)){
		ROS_INFO("Client Succeded");
	}

}

void MyCommonViz::parseMessage(navigation_msgs::TopologicalMap& map, TopoMapMetaPtr msg){
  map.header.frame_id = msg->lookupString("frame_id");
  map.map_id = msg->lookupString("map_id");
  int nnodes = msg->lookupInt("nsize");

  for (auto i=0; i<nnodes;i++){
    navigation_msgs::TopologicalNode node;
    node.name =  msg->lookupString("node_name_" + std::to_string(i));
    node.map =  msg->lookupString("node_map_" + std::to_string(i));
    node.pose.position.x = msg->lookupDouble("node_x_" + std::to_string(i));
    node.pose.position.y = msg->lookupDouble("node_y_" + std::to_string(i));
    node.pose.position.z = msg->lookupDouble("node_z_" + std::to_string(i));
    node.pose.orientation.x = msg->lookupDouble("node_ox_" + std::to_string(i));
    node.pose.orientation.y = msg->lookupDouble("node_oy_" + std::to_string(i));
    node.pose.orientation.z = msg->lookupDouble("node_oz_" + std::to_string(i));
    node.pose.orientation.w = msg->lookupDouble("node_ow_" + std::to_string(i));

    int nverts = msg->lookupInt("nverts_"+ std::to_string(i));

    for (auto j=0; j<nverts; j++){
      navigation_msgs::Vertex verts;
      verts.x = msg->lookupDouble("node_"+ std::to_string(i)+"_vx_"+std::to_string(j));
      verts.y = msg->lookupDouble("node_"+ std::to_string(i)+"_vy_"+std::to_string(j));
      node.verts.push_back(verts);
    }

    int nedges = msg->lookupInt("nedges_"+ std::to_string(i));

    for (auto v =0; v<nedges ; v++){
      navigation_msgs::Edge edge;
      edge.edge_id=msg->lookupString("node_"+ std::to_string(i)+"_ex_"+std::to_string(v));
      edge.node = msg->lookupString("node_"+ std::to_string(i)+"_ey_"+std::to_string(v));
      node.edges.push_back(edge);
    }

    map.nodes.push_back(node);
  }

  map.info.map_frame=msg->lookupString("info_mapframe");
  map.info.sizex=msg->lookupDouble("info_sizex");
  map.info.sizey=msg->lookupDouble("info_sizey");
  map.info.robot_radius=msg->lookupDouble("info_rrs");
  map.info.direction=msg->lookupInt("info_direction");
  map.info.angle_offset=msg->lookupDouble("info_angleoffset");

}


void MyCommonViz::visualizeMap(){
	//Node Creation
	node_map_.clear();
	visualization_msgs::Marker temporal_marker;

	marker_array_.markers.clear();
	edges_.clear();

	//For now this fields are constants FOR NODES
	temporal_marker.header.frame_id= map_frame_;
	temporal_marker.header.stamp = ros::Time::now();
	temporal_marker.ns = "nodes"; //TODO maybe add segmentation layers
	temporal_marker.type = visualization_msgs::Marker::CUBE;

	//DELETE PREVIOUS
	temporal_marker.action = visualization_msgs::Marker::DELETEALL;
	marker_array_.markers.push_back(temporal_marker);
	map_publisher_.publish(marker_array_);

	//Create New Nodes
	temporal_marker.action = visualization_msgs::Marker::ADD;
	temporal_marker.scale.x = 0.1;//robot_radius_/5;//divided by three just o see edges
	temporal_marker.scale.y = 0.1;//robot_radius_/5;
	temporal_marker.scale.z = 0.1;
	temporal_marker.color.g = 0.3;
	temporal_marker.color.a = 0.5;

	temporal_marker.pose.orientation.w = 1.0;

	//For edges
	geometry_msgs::Point temporal_point;
	visualization_msgs::Marker temporal_edges;

	temporal_edges.header.frame_id=map_frame_;
	temporal_edges.header.stamp = ros::Time::now();
	temporal_edges.ns = "edges"; //TODO maybe add segmentation layers
	temporal_edges.type = visualization_msgs::Marker::LINE_LIST;
	temporal_edges.action = visualization_msgs::Marker::ADD;
	temporal_edges.scale.x = 0.1;
	temporal_edges.scale.y = 0.1;
	temporal_edges.scale.z = 0.1;
	temporal_edges.color.r = 1.0;
	temporal_edges.color.g = 1.0;
	temporal_edges.color.a = 1.0;
	temporal_edges.pose.orientation.w = 1.0;


	std::vector<std::pair<float,float> > vector;
	map_utils_->calculateCenters(vector,  nrows_, y_cells_, 2*robot_radius_*direction_, (terrain_y_)/(default_npoints_-1));

	int id, index_1, index_2 = 0;
	int col;

	float tx = 0.0;
	float ty = 0.0;
	float tx1 = 0.0;
	float ty1 = 0.0;

	//TODO VISUALIZE ALL and current row
	for (int current_row = 0; current_row < nrows_; current_row++){
		int min_index = current_row*y_cells_;
		int max_index = (current_row*y_cells_) + y_cells_;
		double yaw =(current_row%2) ? -1.57 : 1.57;
		yaw+=angle_;

		for( auto id = min_index; id< max_index; ++id){
			//Storing Nodes
			//col = id/y_cells_;
			temporal_marker.id = id;
			tx = vector[id].first;
			ty = vector[id].second;
			/*
			if (direction_==1){
				tx = -tx;
			}
			*/
			temporal_marker.pose.position.x = tx * cos(angle_) - ty* sin(angle_);
			temporal_marker.pose.position.y = tx * sin(angle_) + ty* cos(angle_);

			tf2::Quaternion quat_tf;
			quat_tf.setRPY(0.0, 0.0, yaw);
			geometry_msgs::Quaternion quat_msg;
			tf2::convert(quat_tf, temporal_marker.pose.orientation);

			marker_array_.markers.push_back(temporal_marker);

			std::string id_str("error");
			std::string next_id_str("error");

			id_str ="node_" + std::to_string(id);
			next_id_str ="node_" + std::to_string(id+1);

			node_map_[id_str] = temporal_marker.pose;

			if (id == max_index-1){
				continue;
			}

			temporal_edges.id = 100+id;

			tx1 = vector[id+1].first;
			ty1 = vector[id+1].second;

			/*
			if (direction_==1){
				tx1 = -tx1;
			}
			*/
			temporal_point.x = tx * cos(angle_) - ty* sin(angle_);
			temporal_point.y = tx * sin(angle_) + ty* cos(angle_);

			/*
			if (direction_==1){
				temporal_point.x = - temporal_point.x;
			}
			*/

			temporal_edges.points.push_back(temporal_point);

			temporal_point.x = tx1 * cos(angle_) - ty1* sin(angle_);
			temporal_point.y = tx1 * sin(angle_) + ty1* cos(angle_);

			//Marker
			/*
			if (direction_==1){
				temporal_point.x = - temporal_point.x;
			}
			*/

			temporal_edges.points.push_back(temporal_point);

			edges_.emplace_back(id_str, next_id_str);
			edges_.emplace_back(next_id_str,id_str);

			marker_array_.markers.push_back(temporal_edges);
		}
	}

	/*
	for (auto e : edges_){
	std::cout << e.first << " to " << e.second <<std::endl;
	}
	*/

	map_publisher_.publish(marker_array_);
	publishRegion();
}


void MyCommonViz::publishRegion(){
	visualization_msgs::Marker region;
	region.header.frame_id = map_frame_;
	region.ns = "region";
	region.id = 20001;
	region.type = visualization_msgs::Marker::LINE_STRIP;
	region.action = visualization_msgs::Marker::DELETE;

	region_publisher_.publish(region);
	region.action = visualization_msgs::Marker::ADD;
	region.scale.x = 0.1;
	region.scale.y = 0.1;
	region.scale.z = 0.1;
	region.color.r = 0.0;
	region.color.g = 1.0;
	region.color.a = 1.0;

	float tx = 0.0;
	float ty = 0.0;

	//primver punto de la region
	geometry_msgs::Point p;
	float offset = 2.0;

	tx = -(offset)*direction_;
	ty = -offset;
	p.x = tx * cos(angle_) - ty *sin(angle_);
	p.y = tx * sin(angle_) + ty *cos(angle_);
	p.z = 0.0;
	region.points.push_back(p);

	//Segundo punto
	tx = (terrain_x_)*direction_;
	ty = -offset;
  	p.x = tx * cos(angle_) - ty *sin(angle_);
	p.y = tx * sin(angle_) + ty *cos(angle_);
	p.z = 0.0;
	region.points.push_back(p);

	//Tercer punto
	tx = (terrain_x_)*direction_;
	ty = terrain_y_ + offset;
	p.x = tx * cos(angle_) - ty *sin(angle_);
	p.y = tx * sin(angle_) + ty *cos(angle_);
	p.z = 0.0;
	region.points.push_back(p);

	//Cuarto punto
	tx = -(offset)*direction_;
	ty = terrain_y_ + offset;
	p.x = tx * cos(angle_) - ty *sin(angle_);
	p.y = tx * sin(angle_) + ty *cos(angle_);
	p.z = 0.0;
	region.points.push_back(p);

	//Primer punto
	tx = -(offset)*direction_;
	ty = -offset;
	p.x = tx * cos(angle_) - ty *sin(angle_);
	p.y = tx * sin(angle_) + ty *cos(angle_);
	p.z = 0.0;
	region.points.push_back(p);
	region_publisher_.publish(region);
}

// Destructor.
MyCommonViz::~MyCommonViz(){
  map_publisher_.shutdown();
  region_publisher_.shutdown();
  nh_.shutdown();
  //delete map_utils_;
  delete controls_layout_;
  delete main_layout_;
  delete render_panel_;
  delete message_store_;
  delete manager_;
}
