#include <offline.h>

using namespace gr_control_gui;
// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : MyCommonViz( parent ), current_row_(1),
      id_maxnumberrows_(1) {
  ROS_INFO("OFFLINE CONTRUCTOR");
  // Construct and lay out labels and slider controls.
  QLabel* width_label = new QLabel("Y Terrain" );
  QSlider* width_slider = new QSlider( Qt::Horizontal );
  width_slider->setMinimum( 1.00 );
  width_slider->setMaximum( 100.0 );

  QLabel* height_label = new QLabel( "X Terrain" );
  QSlider* height_slider = new QSlider( Qt::Horizontal );
  QSlider* angle_slider = new QSlider( Qt::Horizontal );

  height_slider->setMinimum( 1.0 );
  height_slider->setMaximum( 100.0 );

  QLabel* angle_label = new QLabel( "Angle " );
  angle_slider->setMinimum(-180);
  angle_slider->setMaximum(180);

  x_text_ = new QTextEdit(QString("0.0"));
  x_text_->setReadOnly(true);
  x_text_->setMaximumSize(QSize(100, 50));

  y_text_ = new QTextEdit(QString("0.0"));
  y_text_->setReadOnly(true);
  y_text_->setMaximumSize(QSize(100, 50));

  angle_text_ = new QTextEdit(QString("0.0"));
  angle_text_->setReadOnly(true);
  angle_text_->setMaximumSize(QSize(100, 50));

  checkbox_ = new QCheckBox("direction", this);

  QPushButton* save_topological_map = new QPushButton ("Store Map");
  QPushButton* delete_topological_map = new QPushButton ("Delete Map");
  QPushButton* execute_map = new QPushButton ("Execute Map");


  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( width_label, 0, 0 );
  controls_layout->addWidget( width_slider, 0, 1 );
  controls_layout->addWidget( y_text_, 0, 2);

  controls_layout->addWidget( height_label, 1, 0 );
  controls_layout->addWidget( height_slider, 1, 1 );
  controls_layout->addWidget( x_text_, 1, 2);

  controls_layout->addWidget( angle_label, 2, 0 );
  controls_layout->addWidget( angle_slider, 2, 1 );
  controls_layout->addWidget( angle_text_, 2, 2);

  controls_layout->addWidget( save_topological_map, 3, 0 );
  controls_layout->addWidget( delete_topological_map, 3, 1);
  controls_layout->addWidget( checkbox_, 3, 2);


  QLabel* map_frame_label = new QLabel("Map Frame");
  QLineEdit* map_frame_edit = new QLineEdit();
  map_frame_edit->setText(QString("map"));
  map_frame_ = "map";
  controls_layout->addWidget( map_frame_label, 5, 0 );
  controls_layout->addWidget( map_frame_edit, 5, 1 );

  // Construct and lay out render panel.
  //render_panel_ = new rviz::RenderPanel();
  main_layout_->addLayout( controls_layout );

  // Set the top-level layout for this MyViz widget.
  //setLayout( main_layout );

  // Make signal/slot connections.
  connect( width_slider, SIGNAL( valueChanged( int )), this, SLOT( setTerrainY(  int )));
  connect( height_slider, SIGNAL( valueChanged( int )), this, SLOT( setTerrainX(  int)));
  connect( angle_slider, SIGNAL( valueChanged( int )), this, SLOT( setAngle(  int )));

  connect( save_topological_map, SIGNAL( released( )), this, SLOT( saveMap( )));
  connect( delete_topological_map, SIGNAL( released( )), this, SLOT( deleteTopoMap( )));

  connect( map_frame_edit, SIGNAL(textChanged(QString)), this, SLOT(setFrame(QString)));
  connect( checkbox_, SIGNAL(stateChanged(int )), this, SLOT(setDirection( int )));

  // Initialize the slider values.
  height_slider->setValue( 2.0 );
  width_slider->setValue( 2.0 );

  manager_->setFixedFrame(map_frame_.c_str());
  manager_->initialize();
  manager_->startUpdate();
  update_server_ = nh_.advertiseService("/topological/edges", &MyViz::setEdges, this);
  //server_ = new boost::make_shared<actionlib::SimpleActionServer<GREdgesAction>>(nh_, "/topological/edges", boost::bind(&MyViz::execute_cb, _1), false);
  //server_->start();

}

bool MyViz::setEdges(gr_action_msgs::GREdges2::Request& req,gr_action_msgs::GREdges2::Response& res){
 ROS_INFO("Edges received");
  terrain_y_ = req.width_meters;
  y_cells_ = 10;//ceil(terrain_y_/2);

  terrain_x_ = req.height_meters;
  x_cells_ = ceil(terrain_x_/1);
  visualizeMap();
  return true;
}


MyViz::~MyViz()
{
  ROS_ERROR("D offline init");
  ROS_ERROR("D offline end");

}
/*
void MyViz::execute_cb(const GREdgesActionGoal& goal){
  ROS_INFO("PLAN received");
  terrain_y_ = goal->height_meters;
  y_cells_ = ceil(terrain_y_/1);

  terrain_x_ = goal.width_meters;
  x_cells_ = ceil(terrain_x_/1);
  visualizeMap();

  server_->setSucceeded();

}
*/

void MyViz::setDirection(int state){
  std::cout << "direction " << state << std::endl;
  direction_ = state-1;
  visualizeMap();
}

void MyViz::setFrame(QString frame){
  map_frame_ = frame.toStdString();
}

void MyViz::setTerrainY( int value){
  terrain_y_ = value;
  y_text_->setText(std::to_string(value).c_str());
  y_cells_ = 10;//ceil(value/4);
  visualizeMap();
}

void MyViz::setTerrainX( int value ){
  terrain_x_ = value;
  x_text_->setText(std::to_string(value).c_str());
  x_cells_ = ceil(value/1);
  visualizeMap();
}

void MyViz::setAngle( int value ){
  angle_ = M_PI*value/180;
  angle_text_->setText(std::to_string(angle_).c_str());
  visualizeMap();
}

void MyViz::deleteTopoMap(){
    if (storing_id_.empty()){
      std::cout << "Map not detected" << std::endl;
      return;
    }
  	message_store_->deleteID(storing_id_);
    std::cout << "deleted "<< storing_id_ << std::endl;
    storing_id_ = "";
    std::remove("/tmp/lastmap_id.txt");
}

void MyViz::saveMap(){
  navigation_msgs::TopologicalMap topo_map;
  navigation_msgs::TopologicalNode topo_node;
  navigation_msgs::Vertex vertex;

  navigation_msgs::Edge edge;
  std::string map_id("wish_map_move_base");
  //deleteTopoMap();
  topo_map.header.frame_id = map_frame_;
  topo_map.map_id = map_id;

  for (auto const & node : node_map_){

    topo_node.edges.clear();
    topo_node.verts.clear();
    topo_node.pose = node.second;
    topo_node.name = node.first;

    vertex.x = -robot_radius_/2;
    vertex.y = robot_radius_/2;

    topo_node.verts.push_back(vertex);
    vertex.x = robot_radius_/2;
    vertex.y = robot_radius_/2;

    topo_node.verts.push_back(vertex);
    vertex.x = robot_radius_/2;
    vertex.y = -robot_radius_/2;
    topo_node.verts.push_back(vertex);
    vertex.x = -robot_radius_/2;
    vertex.y = -robot_radius_/2;
    topo_node.verts.push_back(vertex);

    for (Edges & e : edges_){
      if (e.first.compare(node.first)==0){
        edge.edge_id = e.first + "_" + e.second;
        edge.node = e.second;
        //At the moment deprected
        //edge.action = "sbpl_action";
        topo_node.edges.push_back(edge);
      }
    }
   
    topo_map.nodes.push_back(topo_node);
  }

  topo_map.info.map_frame = map_frame_;
  topo_map.info.sizex = terrain_x_;
  topo_map.info.sizey = terrain_y_;
  topo_map.info.robot_radius = robot_radius_;
  topo_map.info.direction = direction_;
  topo_map.info.angle_offset = angle_;

  std::ifstream in("/tmp/lastmap_id.txt");
  //out << storing_id_;
  in >> storing_id_;
  in.close();  

  if (!storing_id_.empty()){
    std::vector<boost::shared_ptr<navigation_msgs::TopologicalMap > >aaa;
    message_store_->queryID<navigation_msgs::TopologicalMap>(storing_id_,aaa);
    message_store_->updateNamed(map_id, topo_map);
    std::cout<<"Map \""<<map_id<<"\" updated with id "<<storing_id_<<std::endl;
  }
  else{
    storing_id_ = message_store_->insertNamed(map_id, topo_map);
    std::cout<<"Map \""<<map_id<<"\" inserted with id "<<storing_id_<<std::endl;
  }
  std::ofstream out("/tmp/lastmap_id.txt");
  out << storing_id_;
  out.close();
  //
  id_maxnumberrows_ = (2*terrain_x_/robot_radius_)-1;
  std::cout << "Max Rows Idx "<< id_maxnumberrows_ << std::endl;
}