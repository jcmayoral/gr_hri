#include <common.h>

using namespace gr_control_gui;

MyCommonViz::MyCommonViz( QWidget* parent): QWidget( parent ), nh_{},  robot_radius_(1.5),
                 x_cells_{1}, y_cells_{1}, terrain_x_(1.0), terrain_y_(1.0), id_maxnumberrows_(1),
                 angle_{0.0}, direction_{-1}{
  ROS_INFO("COMMON CONTRUCTOR");
  map_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("full_topological_map", 1 );
  region_publisher_ = nh_.advertise<visualization_msgs::Marker>("region", 1 );
  //collection and database as arguments to messageStoreProxy
  message_store_ = new mongodb_store::MessageStoreProxy(nh_,"topological_maps","message_store");
  main_layout_ = new QVBoxLayout();
  render_panel_ = new rviz::RenderPanel();
  controls_layout_ = new QGridLayout();
  QPushButton* load_topological_map = new QPushButton ("Load Map");
  controls_layout_->addWidget( load_topological_map, 0, 0 );
  connect( load_topological_map, SIGNAL( released( )), this, SLOT( loadMap()));


  manager_ = new rviz::VisualizationManager( render_panel_ );
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

  rviz::Display* pc;
  pc = manager_->createDisplay( "rviz/PointCloud2", "velodyne_pc", true );
  ROS_ASSERT( pc != NULL );
  pc->subProp( "Topic" )->setValue("/velodyne_points");


  rviz::Display* dect;
  dect = manager_->createDisplay( "jsk_rviz_plugin/BoundingBoxArray", "dect_bb", true );
  ROS_ASSERT( dect != NULL );
  dect->subProp( "Topic" )->setValue("/detection/bounding_boxes");  

  manager_->setFixedFrame("map");
}

void MyCommonViz::loadGUI(){
  ROS_ERROR("LoadGUI");
  manager_->initialize();
  manager_->startUpdate();
  main_layout_->addLayout( controls_layout_ );
  main_layout_->addWidget( render_panel_ );
  setLayout( main_layout_ );
}

void MyCommonViz::loadMap(){
  ROS_INFO("LOAD MAP");

  std::ifstream in("/tmp/lastmap_id.txt");
  //out << storing_id_;
  in >> storing_id_;
  in.close();

  std::string map_id("wish_map_move_base");
  if (!storing_id_.empty()){
    std::cout << "HERE " << storing_id_ << std::endl;
    std::vector< boost::shared_ptr<navigation_msgs::TopologicalMap> > results_map;

    if(message_store_->queryNamed<navigation_msgs::TopologicalMap>(map_id,results_map)){
      //message_store_->updateNamed(map_id, topo_map);
      //std::cout << results_map.size() << std::endl;
      BOOST_FOREACH( boost::shared_ptr<  navigation_msgs::TopologicalMap> map,  results_map){
        load_map_ = *map;
      }
      ROS_WARN_STREAM(load_map_.info);

      angle_ = load_map_.info.angle_offset;
      direction_ = load_map_.info.direction;
      map_frame_ = load_map_.info.map_frame;
      robot_radius_ = load_map_.info.robot_radius;
      terrain_y_ = load_map_.info.sizey;
      terrain_x_ = load_map_.info.sizex;
      x_cells_ =  ceil(terrain_x_/1);
      y_cells_ =  10;//ceil(terrain_y_/1);
      id_maxnumberrows_ = x_cells_-1;

      manager_->setFixedFrame(map_frame_.c_str());

      ROS_ERROR("YEI");
      visualizeMap();
      return;
    }


     std::vector< boost::shared_ptr<navigation_msgs::TopologicalNode> > results_node;
        //On this version the map is stored by NAME Not anymore nodes stored
        ROS_WARN("QUERY NODES");
        if(message_store_->queryNamed<navigation_msgs::TopologicalNode>(map_id, results_node, false)) {
            load_map_.nodes.clear();
            navigation_msgs::TopologicalNode node;
            BOOST_FOREACH( boost::shared_ptr<navigation_msgs::TopologicalNode> node,  results_node){
                ROS_DEBUG_STREAM("Got by name: " << *node);
                load_map_.nodes.push_back(*node);
            }
           return;
        }

    std::cout<<"Map aaaaa "<<map_id<< " failed to load with id "<<storing_id_<<std::endl;
    
    /*
    if(message_store_->queryNamed<navigation_msgs::TopologicalMap>(map_id, results_map)) {
      ROS_INFO("AAAA");
      load_map_.nodes.clear();
      std::cout << results_map.size() << std::endl;
      BOOST_FOREACH( boost::shared_ptr<  navigation_msgs::TopologicalMap> map,  results_map){
        ROS_INFO_STREAM("Got by name: " << *map);
        load_map_ = *map;
      }
      ROS_INFO_STREAM(load_map_);
      return;
    }
    */
  }

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
  //map_publisher_.publish(marker_array_);

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

  map_utils_->calculateCenters(vector,  x_cells_, y_cells_, direction_*1.0, (terrain_y_-robot_radius_)/9.0);

  int id, index_1, index_2 = 0;
  int col;

  float tx = 0.0;
  float ty = 0.0;
  float tx1 = 0.0;
  float ty1 = 0.0;

  //TODO VISUALIZE ALL and current row 
  for (int current_row = 0; current_row < x_cells_; current_row++){
    int min_index = current_row*y_cells_;
    int max_index = (current_row*y_cells_) + y_cells_;
    double yaw =(current_row%2) ? -1.57 : 1.57;
    yaw+=angle_;

    //std::cout << "start " << min_index << " end " << max_index << std::endl;

    for( auto id = min_index; id< max_index; ++id){
      //Storing Nodes
      col = id/y_cells_;
      temporal_marker.id = id;
      tx = vector[id].first;
      ty = vector[id].second;
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
      //std::cout << id_str << " NODE STRING " << std::endl;
      next_id_str ="node_" + std::to_string(id+1);

      /*
      //Nasty Hack
      if (id == min_index){
        id_str = "start_node";
        if (col%2 == 1){
          id_str = "end_node";
        }
      }
      else if(id == max_index-1){
        id_str = "end_node";
        if (col%2 == 1){
          id_str = "start_node";
        }
      }
      else{
        id_str ="node_" + std::to_string(id);
      }

      if ((id+1) == min_index){
        next_id_str = "start_node";
        if (col%2 == 1){
          next_id_str = "end_node";
        }
      }
      else if(id == (max_index-2)){
        next_id_str = "end_node";
        if (col%2 == 1){
          next_id_str = "start_node";
        }
      }
      else{
        next_id_str ="node_" + std::to_string(id+1);
      }
      */
      //end of nasty hack
      node_map_[id_str] = temporal_marker.pose;
      //ROS_ERROR_STREAM("FINAL NODE NAME " << id_str);

      if (id == max_index-1){
        //skip edges of last node of the row
        continue;
      }

      temporal_edges.id = 100+id;

      tx1 = vector[id+1].first;
      ty1 = vector[id+1].second;
      temporal_point.x = tx * cos(angle_) - ty* sin(angle_);
      temporal_point.y = tx * sin(angle_) + ty* cos(angle_);
      temporal_edges.points.push_back(temporal_point);
      temporal_point.x = tx1 * cos(angle_) - ty1* sin(angle_);
      temporal_point.y = tx1 * sin(angle_) + ty1* cos(angle_);
      //Marker
      temporal_edges.points.push_back(temporal_point);
      //temporal_edges.points.push_back(temporal_point);
      //Edges ids

      //birectional
      //std::cout << id_str << next_id_str << std::endl;
      edges_.emplace_back(id_str, next_id_str);
      //edges_.emplace_back(next_id_str,id_str);

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

  geometry_msgs::Point p;

  tx = -robot_radius_*direction_;
  ty = -robot_radius_;
  p.x = tx * cos(angle_) - ty *sin(angle_);
  p.y = tx * sin(angle_) + ty *cos(angle_);
  p.z = 0.0;
  region.points.push_back(p);


  tx = (terrain_x_ + robot_radius_)*direction_;
  ty = -robot_radius_;

  p.x = tx * cos(angle_) - ty *sin(angle_);
  p.y = tx * sin(angle_) + ty *cos(angle_);
  p.z = 0.0;
  region.points.push_back(p);

  tx = (terrain_x_ + robot_radius_)*direction_;
  ty = terrain_y_ + robot_radius_;
  p.x = tx * cos(angle_) - ty *sin(angle_);
  p.y = tx * sin(angle_) + ty *cos(angle_);
  p.z = 0.0;
  region.points.push_back(p);


  tx = -robot_radius_*direction_;
  ty = terrain_y_ + robot_radius_;
  p.x = tx * cos(angle_) - ty *sin(angle_);
  p.y = tx * sin(angle_) + ty *cos(angle_);
  p.z = 0.0;
  region.points.push_back(p);
  
  tx = -robot_radius_*direction_;
  ty = -robot_radius_;
  p.x = tx * cos(angle_) - ty *sin(angle_);
  p.y = tx * sin(angle_) + ty *cos(angle_);
  p.z = 0.0;
  region.points.push_back(p);

  region_publisher_.publish(region);

}

// Destructor.
MyCommonViz::~MyCommonViz(){
  ROS_ERROR("D common init");
  map_publisher_.shutdown();
  region_publisher_.shutdown();
  nh_.shutdown();
  //delete map_utils_;
  delete controls_layout_;
  delete main_layout_;
  delete render_panel_;
  delete message_store_;
  delete manager_;
  ROS_ERROR("D common end");
}
