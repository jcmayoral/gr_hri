#include <online.h>

using namespace gr_control_gui;
// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : MyCommonViz( parent ), current_row_(1), task_{"CUT"},
   gr_action_client_("gr_simple_manager", true), nviapoints_{9}, mode_{1}, span_{1}, cancel_goal_{false}
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


  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( column_label, 2, 0 );
  controls_layout->addWidget( column_spinbox, 2, 1 );


  QSpinBox* via_spinbox = new QSpinBox;
  via_spinbox->setRange(0, 50);
  via_spinbox->setSingleStep(1);
  via_spinbox->setValue(nviapoints_);
  controls_layout->addWidget( new QLabel("Number of ViaPoints"), 2, 2 );
  controls_layout->addWidget( via_spinbox, 2, 3 );


  QLabel* mode_label = new QLabel("Mode Selector");
  //QListView* mode_selector = new QListView;
  //mode_selector.addItem(QString("mode 0"));
  mode_selector_ = new QListWidget;
  mode_selector_->addItems(QStringList({"Visit All","Just End","Visit Some"}));
  mode_selector_->setMaximumHeight(100);
  mode_selector_->setCurrentRow(1);
  //mode_selector,addItem(QString("VISIT ALL"));
  //mode_selector,addItem(QString("VISIT ALL"));
  //mode_selector,addItem(QString("VISIT ALL"));

  controls_layout->addWidget( mode_label, 3, 0 );
  controls_layout->addWidget( mode_selector_, 3, 1 );


  QLabel* task_label = new QLabel("Task Selector");
  //QListView* mode_selector = new QListView;
  //mode_selector.addItem(QString("mode 0"));
  task_selector_ = new QListWidget;
  task_selector_->addItems(QStringList({"Cut","Collect"}));
  task_selector_->setMaximumHeight(100);
  task_selector_->setCurrentRow(0);
  //mode_selector,addItem(QString("VISIT ALL"));
  //mode_selector,addItem(QString("VISIT ALL"));
  //mode_selector,addItem(QString("VISIT ALL"));

  controls_layout->addWidget( task_label, 3, 2 );
  controls_layout->addWidget( task_selector_, 3, 3 );

  QPushButton* execute_map = new QPushButton ("Execute Nav");
  QPushButton* stop_map = new QPushButton ("Stop Nav");

  checkbox_ = new QCheckBox("Resume last execution", this);
  controls_layout->addWidget( checkbox_, 4, 0 );
  controls_layout->addWidget( execute_map, 4, 1 );
  controls_layout->addWidget( stop_map, 4, 2 );


  QLabel* time_to_go_label = new QLabel("Current Row ");
  time_to_go = new QLabel("0");
  QFont f( "Arial", 30, QFont::Bold);
  time_to_go->setFont(f);
  time_to_go->setFixedHeight(50);
  //time_to_go->setFixedWidth(50);

  controls_layout->addWidget( time_to_go_label, 5, 0 );
  controls_layout->addWidget( time_to_go, 5, 1 );


  span_spinbox_ = new QSpinBox;
  span_spinbox_->setRange(0, nviapoints_);
  span_spinbox_->setSingleStep(1);
  span_spinbox_->setValue(span_);
  controls_layout->addWidget( new QLabel("Skip N ViaPoints"), 5, 2 );
  controls_layout->addWidget( span_spinbox_, 5, 3 );

  // Construct and lay out render panel.
  main_layout_->addLayout( controls_layout );

  // Set the top-level layout for this MyViz widget.
  //setLayout( main_layout );

  // Make signal/slot connections.
  connect( execute_map, SIGNAL( released( )), this, SLOT( executeTopoMap( )));
  connect( stop_map, SIGNAL( released( )), this, SLOT( stopExecution( )));

  connect( column_spinbox, SIGNAL(valueChanged(int)), this, SLOT(setDesiredRow(int)));
  connect( mode_selector_, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(setMode(QListWidgetItem*)));
  connect( task_selector_, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(setTask(QListWidgetItem*)));

  connect( via_spinbox, SIGNAL(valueChanged(int)), this, SLOT(setNViaPoints(int)));
  connect( span_spinbox_, SIGNAL(valueChanged(int)), this, SLOT(setSpan(int)));

  /*
  manager_->setFixedFrame("workspace");
  manager_->initialize();
  manager_->startUpdate();
  */
  update_client_ = local_nh.serviceClient<gr_map_utils::UpdateMap>("update_metric_map");
  //time_to_go_sub_ = local_nh.subscribe("/gr_sbpl_trajectory_generator_node/time_to_go", 1, &MyViz::timetogoCB, this);
}

void MyViz::feedbackCb(const gr_action_msgs::GRNavigationFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM(*feedback);
}


void MyViz::setNViaPoints(int nvia){
  span_spinbox_->setRange(0, nviapoints_);
  span_spinbox_->setValue(1);
  nviapoints_ = nvia;
}

void MyViz::setSpan(int span){
  span_ = span;
}

void MyViz::stopExecution(){
  std::cout << "calling stop" << std::endl;
  cancel_goal_ = true;
  //if gui is terminated .. better to cancel all goals
  gr_action_client_.cancelAllGoals();
}

void MyViz::timetogoCB(const std_msgs::Float32ConstPtr time2go){
  time_to_go -> setText(QString::number(time2go->data));
}

// Destructor.
MyViz::~MyViz()
{
  //deleteTopoMap();
  time_to_go_sub_.shutdown();
  online_map_publisher_.shutdown();
  //region_publisher_.shutdown();
  reset_publisher_.shutdown();
  update_client_.shutdown();

}

void MyViz::setTask(QListWidgetItem* item ){
  task_ = item->text().toStdString();
  std::cout << "CHANGE Task TO "<< task_ << std::endl;
}


void MyViz::setMode(QListWidgetItem* item ){
  mode_ = mode_selector_->row(item);
  std::cout << "CHANGE MODE TO "<< mode_ << std::endl;
}

void MyViz::setFrame(QString frame){
  map_frame_ = frame.toStdString();
}

void MyViz::setDesiredRow(int row){
  id_maxnumberrows_ = nrows_-1;

  if (row < id_maxnumberrows_){
    int min, max;
    current_row_ = std::min(id_maxnumberrows_, row);
    visualizeRowMap(current_row_, min , max);
  }
  else{
    ROS_ERROR("ERROR");
  }
}

void MyViz::executeTopoMap(){
  cancel_goal_ = false;
  //std::thread worker_thread();
  t1 = new std::thread(&MyViz::executeCycle, this, 0);
  t1->detach();
  ROS_INFO("MOTION EXECUTION FINISHED");
}



void MyViz::executeCycle(int cycle){
  current_row_ = cycle;
  ROS_INFO_STREAM("current row "<< cycle);
  std::string mystr = std::to_string(cycle) + " of " + std::to_string(id_maxnumberrows_);
  time_to_go -> setText( QString(mystr.c_str()));

  //deleteTopoMap();
  //ros::Duration(1.0).sleep();

  //BUG
  //visualizeMap();
  int start_node = 0;
  int end_node = 1;
  visualizeRowMap(current_row_, start_node, end_node);


  /*this is teh fancy topological map
  reset_publisher_.publish(std_msgs::Time());
  */
  ros::Duration(1.0).sleep();
  gr_action_msgs::GRNavigationGoal goal;
  goal.mode = mode_;
  goal.task_id = task_;
  goal.span = span_;
  goal.row_id = current_row_;
  goal.plan = online_marker_array_;
  goal.start_node = "node_" + std::to_string(start_node);
  goal.goal_node = "node_" + std::to_string(end_node);
  //goal.start_node = std::string("start_node").c_str();
  gr_action_client_.sendGoal(goal, MyClient::SimpleDoneCallback(),
              MyClient::SimpleActiveCallback(),
              boost::bind(&MyViz::feedbackCb, this, _1));


  bool finished_before_timeout = gr_action_client_.waitForResult();
  actionlib::SimpleClientGoalState state = gr_action_client_.getState();
  ROS_INFO("Action finished: %s",state.toString().c_str());
  if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED ||
      state == actionlib::SimpleClientGoalState::StateEnum::REJECTED){
    ROS_ERROR("Finished with error ");
    return;
    cycle = id_maxnumberrows_ + 1;
  }
  //Update map if topological + metric map is used
  /*
  gr_map_utils::UpdateMap req;
  if(update_client_.call(req)){
    ROS_INFO("Client Succeded");
  }
  */
  /*boost::shared_ptr<std_msgs::Empty const> msg_pointer;
  msg_pointer =  ros::topic::waitForMessage<std_msgs::Empty>("/end_motion");
  */

  if (cycle < id_maxnumberrows_){
    ROS_INFO_STREAM("ROW " << cycle << " of " << id_maxnumberrows_ << "FINISHED ");
    execution_status_.last_row = cycle;
    executeCycle(cycle + 1);
  }
}


void MyViz::visualizeRowMap(int row, int& start_node, int& goal_node){
  //Node Creation
  node_map_.clear();
  visualization_msgs::Marker temporal_marker;

  online_marker_array_.markers.clear();
  //edges_.clear();

  //For now this fields are constants FOR NODES
  temporal_marker.header.frame_id= map_frame_;
  temporal_marker.header.stamp = ros::Time::now();
  temporal_marker.ns = "nodes"; //TODO maybe add segmentation layers
  temporal_marker.type = visualization_msgs::Marker::ARROW;

  //DELETE PREVIOUS
  temporal_marker.action = visualization_msgs::Marker::DELETEALL;
  online_marker_array_.markers.push_back(temporal_marker);
  online_map_publisher_.publish(online_marker_array_);
  online_marker_array_.markers.clear();

  //Create New Nodes
  temporal_marker.action = visualization_msgs::Marker::ADD;
  temporal_marker.scale.x = robot_radius_;//divided by three just o see edges
  temporal_marker.scale.y = robot_radius_/5;
  temporal_marker.scale.z = 0.25;
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
  temporal_edges.scale.x = 0.1;//robot_radius_/3;
  temporal_edges.scale.y = 0.1;
  temporal_edges.scale.z = 1.5;
  temporal_edges.color.r = 1.0;
  temporal_edges.color.b = 1.0;
  temporal_edges.color.a = 1.0;
  temporal_edges.pose.orientation.w = 1.0;


  std::vector<std::pair<float,float> > vector;

  //x_cells => column
  map_utils_->calculateCenters(vector,  nrows_, nviapoints_, 2*robot_radius_*direction_*1.0, (terrain_y_-robot_radius_)/(nviapoints_-1));

  int id, index_1, index_2 = 0;
  int col;

  //TODO VISUALIZE ALL and current row
  int min_index = row*nviapoints_;
  int max_index = min_index + nviapoints_ -1;

  //int max_index = min_index + nviapoints_;

  //if (row%2){
   // max_index = min_index + nviapoints_;
    //std::swap(min_index, max_index);
  //}

  double yaw =(row%2) ? -1.57 : 1.57;
  yaw+=angle_;
  std::cout << "SIZE OF VECTOR " << vector.size() << std::endl;

  int npointspercolumn = vector.size()/nrows_;
  std::cout <<" points per column " << npointspercolumn << " start index " << npointspercolumn*row << std::endl;


  float tx = 0.0;
  float ty = 0.0;
  float tx1 = 0.0;
  float ty1 = 0.0;

  std::cout << "start " << min_index << " end " << max_index << std::endl;

  for( auto id = min_index; id<= max_index; ++id){
    //Storing Nodes
    std::cout << "COMPUTING " << id <<std::endl;
    std::string id_str("error");
    std::string next_id_str("error");

    id_str ="node_" + std::to_string(id);
    next_id_str ="node_" + std::to_string(id+1);

    col = id/nviapoints_;
    temporal_marker.id = id;
    tx = vector[id].first;
    ty = vector[id].second;
    temporal_marker.pose.position.x = tx * cos(angle_) - ty* sin(angle_);
    temporal_marker.pose.position.y = tx * sin(angle_) + ty* cos(angle_);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat_tf, temporal_marker.pose.orientation);

    std::cout << "COL "<< col <<std::endl;
    //Nasty Hack
    /*
    if (id == min_index){
      id_str = "start_node";
      if (col%2 == 1){
        id_str = "end_node";
      }
    }
    else if(id == max_index){
      id_str = "end_node";
      if (col%2 == 1){
        id_str = "start_node";
      }
    }
    else{
      */
    id_str ="node_" + std::to_string(id);
    //}


    /*
    if ((id+1) == min_index){
      next_id_str = "start_node";
      if (col%2 == 1){
        next_id_str = "end_node";
      }
    }
    else
    */
    /*
    if(id == (max_index-1)){
      next_id_str = "end_node";
      if (col%2 == 1){
        next_id_str = "start_node";
      }
    }
    else{
      */
      next_id_str ="node_" + std::to_string(id+1);
    //}
    //end of nasty hack
    node_map_[id_str] = temporal_marker.pose;
    //ROS_ERROR_STREAM("FINAL NODE NAME " << id_str);

    temporal_marker.text = id_str;
    online_marker_array_.markers.push_back(temporal_marker);

    if (id == max_index){
      //skip edges of last node of the row
     continue;
    }

    temporal_edges.id = 100+id;
    temporal_edges.text = id_str + "::" + next_id_str;
    tx1 = vector[id+1].first;
    ty1 = vector[id+1].second;
    temporal_point.x = tx * cos(angle_) - ty* sin(angle_);
    temporal_point.y = tx * sin(angle_) + ty* cos(angle_);
    temporal_edges.points.push_back(temporal_point);
    temporal_point.x = tx1 * cos(angle_) - ty1* sin(angle_);
    temporal_point.y = tx1 * sin(angle_) + ty1* cos(angle_);
    //Marker
    temporal_edges.points.push_back(temporal_point);
    //Edges ids

    //birectional
    //std::cout << id_str << " TO " <<next_id_str << std::endl;
    //edges_.emplace_back(id_str, next_id_str);
    //edges_.emplace_back(next_id_str,id_str);

    online_marker_array_.markers.push_back(temporal_edges);
  }

  goal_node = (row%2) ? min_index : max_index;
  start_node = (row%2) ? max_index : min_index;;

  online_map_publisher_.publish(online_marker_array_);
}

void MyViz::updateAfterLoad() {
  std::cout << "UPDate AfterLoad on line " << std::endl;
}
