#include <online.h>

using namespace gr_control_gui;
// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : MyCommonViz( parent ), current_row_(1),
    id_maxnumberrows_(1)
{
  ros::NodeHandle local_nh;
  online_map_publisher_ = local_nh.advertise<visualization_msgs::MarkerArray>("current_topological_map", 1 );
  reset_publisher_ = local_nh.advertise<std_msgs::Time>("update_map", 1);


  rviz::Display* marker_display;
  marker_display = manager_->createDisplay( "rviz/MarkerArray", "current_topological_map", true );
  ROS_ASSERT( marker_display != NULL );
  marker_display->subProp( "Marker Topic" )->setValue("current_topological_map");
  //collection and database as arguments to messageStoreProxy
  //message_store_ = new mongodb_store::MessageStoreProxy(local_nh,"topological_maps","message_store");

  QLabel* column_label = new QLabel("Desired Row");
  QSpinBox* column_spinbox = new QSpinBox;
  column_spinbox->setRange(0, 100);
  column_spinbox->setSingleStep(1);
  column_spinbox->setValue(0);

  QPushButton* execute_map = new QPushButton ("Execute Map");

  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( column_label, 2, 0 );
  controls_layout->addWidget( column_spinbox, 2, 1 );
  controls_layout->addWidget( execute_map, 3, 0 );

  QLabel* time_to_go_label = new QLabel("Expected Time To Next Goal");
  time_to_go = new QLabel("0");
  QFont f( "Arial", 30, QFont::Bold);
  time_to_go->setFont(f);
  //time_to_go->setFixedHeight(50);
  //time_to_go->setFixedWidth(50);

  controls_layout->addWidget( time_to_go_label, 4, 0 );
  controls_layout->addWidget( time_to_go, 4, 1 );

  // Construct and lay out render panel.
  main_layout_->addLayout( controls_layout );

  // Set the top-level layout for this MyViz widget.
  //setLayout( main_layout );

  // Make signal/slot connections.
  connect( execute_map, SIGNAL( released( )), this, SLOT( executeTopoMap( )));
  connect( column_spinbox, SIGNAL(valueChanged(int)), this, SLOT(setDesiredRow(int)));
  
  /*
  manager_->setFixedFrame("workspace");
  manager_->initialize();
  manager_->startUpdate();
  */
  update_client_ = local_nh.serviceClient<gr_map_utils::UpdateMap>("update_metric_map");
  time_to_go_sub_ = local_nh.subscribe("/gr_sbpl_trajectory_generator_node/time_to_go", 1, &MyViz::timetogoCB, this);
}

void MyViz::timetogoCB(const std_msgs::Float32ConstPtr time2go){
  time_to_go -> setText(QString::number(time2go->data));
}

// Destructor.
MyViz::~MyViz()
{
  ROS_ERROR("D onlie init");

  //deleteTopoMap();
  time_to_go_sub_.shutdown();
  online_map_publisher_.shutdown();
  //region_publisher_.shutdown();
  reset_publisher_.shutdown();
  update_client_.shutdown();
  ROS_ERROR("D onlie end");

}

void MyViz::setFrame(QString frame){
  map_frame_ = frame.toStdString();
}

void MyViz::setDesiredRow(int row){
  id_maxnumberrows_ = x_cells_;
  std::cout << "MAX " << x_cells_ << std::endl;

  if (row < id_maxnumberrows_){
    current_row_ = std::min(id_maxnumberrows_, row);
    visualizeRowMap(current_row_);
  }
  else{
    ROS_ERROR("ERROR");
  }
}

void MyViz::executeTopoMap(){
  //std::thread worker_thread();
  t1 = new std::thread(&MyViz::executeCycle, this, 0);
  t1->detach();
  ROS_INFO("MOTION EXECUTION FINISHED");
}



void MyViz::executeCycle(int cycle){
  gr_map_utils::UpdateMap req;
  boost::shared_ptr<std_msgs::Empty const> msg_pointer;
  current_row_ = cycle;
  ROS_INFO_STREAM("current row "<< cycle);
  //deleteTopoMap();
  ros::Duration(1.0).sleep();
  visualizeMap();

  reset_publisher_.publish(std_msgs::Time());
  ros::Duration(1.0).sleep();
  if(update_client_.call(req)){
    ROS_INFO("Client Succeded");
  }
  std::cout << "Wait til finish" << std::endl;
  msg_pointer =  ros::topic::waitForMessage<std_msgs::Empty>("/end_motion");

  if (cycle < id_maxnumberrows_){
    ROS_INFO("ROW FINISHED");
    executeCycle(cycle + 1);
  }
}


void MyViz::visualizeRowMap(int row){
  //Node Creation
  node_map_.clear();
  visualization_msgs::Marker temporal_marker;

  online_marker_array_.markers.clear();
  edges_.clear();

  //For now this fields are constants FOR NODES
  temporal_marker.header.frame_id= map_frame_;
  temporal_marker.header.stamp = ros::Time::now();
  temporal_marker.ns = "nodes"; //TODO maybe add segmentation layers
  temporal_marker.type = visualization_msgs::Marker::CUBE;

  //DELETE PREVIOUS
  temporal_marker.action = visualization_msgs::Marker::DELETEALL;
  online_marker_array_.markers.push_back(temporal_marker);
  //map_publisher_.publish(marker_array_);

  //Create New Nodes
  temporal_marker.action = visualization_msgs::Marker::ADD;
  temporal_marker.scale.x = robot_radius_/3;//divided by three just o see edges
  temporal_marker.scale.y = robot_radius_/3;
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
  temporal_edges.scale.x = robot_radius_;
  //temporal_edges.scale.y = 0.1;
  //temporal_edges.scale.z = 1.5;
  temporal_edges.color.r = 1.0;
  temporal_edges.color.b = 1.0;
  temporal_edges.color.a = 1.0;
  temporal_edges.pose.orientation.w = 1.0;


  std::vector<std::pair<float,float> > vector;

  map_utils_->calculateCenters(vector,  x_cells_, y_cells_, 1.0, 1.0);

  int id, index_1, index_2 = 0;
  int col;

  //TODO VISUALIZE ALL and current row 
  int min_index = row*y_cells_;
  int max_index = (row*y_cells_) + y_cells_;
  double yaw =(row%2) ? -1.57 : 1.57;

  std::cout << "start " << min_index << " end " << max_index << std::endl;

  for( auto id = min_index; id< max_index; ++id){
    //Storing Nodes
    col = id/y_cells_;
    temporal_marker.id = id;
    temporal_marker.pose.position.x = vector[id].first;
    temporal_marker.pose.position.y = vector[id].second;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_tf, temporal_marker.pose.orientation);

    online_marker_array_.markers.push_back(temporal_marker);

    std::string id_str("error");
    std::string next_id_str("error");

    id_str ="node_" + std::to_string(id);
    std::cout << id_str << " NODE STRING " << std::endl;
    next_id_str ="node_" + std::to_string(id+1);

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
    //end of nasty hack
    node_map_[id_str] = temporal_marker.pose;
    ROS_ERROR_STREAM("FINAL NODE NAME " << id_str);

    if (id == max_index-1){
      //skip edges of last node of the row
      continue;
    }

    temporal_edges.id = 100+id;
    temporal_point.x = vector[id].first;
    temporal_point.y = vector[id].second;
    temporal_edges.points.push_back(temporal_point);
    temporal_point.x = vector[id+1].first;
    temporal_point.y = vector[id+1].second;
    //Marker
    temporal_edges.points.push_back(temporal_point);
    //Edges ids

    //birectional
    //std::cout << id_str << next_id_str << std::endl;
    edges_.emplace_back(id_str, next_id_str);
    edges_.emplace_back(next_id_str,id_str);

    online_marker_array_.markers.push_back(temporal_edges);
  }

  online_map_publisher_.publish(online_marker_array_);
}
