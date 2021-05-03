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
  current_row_ = std::min(id_maxnumberrows_, row);

  if (row < id_maxnumberrows_){
    visualizeMap();
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
