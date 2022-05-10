#include <offline.h>

using namespace gr_control_gui;
// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : MyCommonViz( parent ), current_row_(1),
      id_maxnumberrows_(1) {
  ROS_INFO("OFFLINE CONTRUCTOR");
  // Construct an<fd lay out labels and slider controls.
  QLabel* width_label = new QLabel("Terrain Length[m]" );
  width_slider_ = new QSlider( Qt::Horizontal );
  width_slider_->setMinimum( 1.00 );
  width_slider_->setMaximum( 200.0 );

  QLabel* height_label = new QLabel( "Terrain Width[m]" );
  height_slider_ = new QSlider( Qt::Horizontal );
  angle_slider_ = new QSlider( Qt::Horizontal );

  height_slider_->setMinimum( 1.0 );
  height_slider_->setMaximum( 200.0 );

  QLabel* angle_label = new QLabel( "Offset Angle[rad]" );
  angle_slider_->setMinimum(-180);
  angle_slider_->setMaximum(180);

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
  QPushButton* update_frame = new QPushButton ("Update Map Frame");


  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( width_label, 0, 0 );
  controls_layout->addWidget( width_slider_, 0, 1 );
  controls_layout->addWidget( y_text_, 0, 2);

  controls_layout->addWidget( height_label, 1, 0 );
  controls_layout->addWidget( height_slider_, 1, 1 );
  controls_layout->addWidget( x_text_, 1, 2);

  controls_layout->addWidget( angle_label, 2, 0 );
  controls_layout->addWidget( angle_slider_, 2, 1 );
  controls_layout->addWidget( angle_text_, 2, 2);


  QLabel* robot_label = new QLabel( "Robot Radius" );
  radius_slider_ = new QSlider( Qt::Horizontal );
  radius_slider_->setMinimum( 1 );
  radius_slider_->setMaximum( 20 );
  radius_slider_->setValue( ceil(robot_radius_*10) );
  controls_layout->addWidget( robot_label, 3, 0 );
  controls_layout->addWidget( radius_slider_, 3, 1 );

  radius_text_ = new QTextEdit(QString(std::to_string(robot_radius_).c_str()));
  radius_text_->setReadOnly(true);
  radius_text_->setMaximumSize(QSize(100, 50));
  controls_layout->addWidget( radius_text_, 3, 2);


  controls_layout->addWidget( save_topological_map, 4, 0 );
  controls_layout->addWidget( delete_topological_map, 4, 1);
  controls_layout->addWidget( checkbox_, 4, 2);


  QLabel* map_frame_label = new QLabel("Map Frame");
  QLineEdit* map_frame_edit = new QLineEdit();
  map_frame_edit->setText(QString("map"));
  map_frame_ = "map";
  controls_layout->addWidget( map_frame_label, 5, 0 );
  controls_layout->addWidget( map_frame_edit, 5, 1 );
  controls_layout->addWidget( update_frame, 5, 2 );


  // Construct and lay out render panel.
  //render_panel_ = new rviz::RenderPanel();
  main_layout_->addLayout( controls_layout );

  // Set the top-level layout for this MyViz widget.
  //setLayout( main_layout );

  // Make signal/slot connections.
  connect( width_slider_, SIGNAL( valueChanged( int )), this, SLOT( setTerrainY(  int )));
  connect( height_slider_, SIGNAL( valueChanged( int )), this, SLOT( setTerrainX(  int)));
  connect( angle_slider_, SIGNAL( valueChanged( int )), this, SLOT( setAngle(  int )));
  connect( radius_slider_, SIGNAL( valueChanged( int )), this, SLOT( setRadius(  int )));

  connect( save_topological_map, SIGNAL( released( )), this, SLOT( saveMap( )));
  connect( delete_topological_map, SIGNAL( released( )), this, SLOT( deleteTopoMap( )));
  connect( update_frame, SIGNAL( released( )), this, SLOT( updateMapFrame( )));


  connect( map_frame_edit, SIGNAL(textChanged(QString)), this, SLOT(setFrame(QString)));
  connect( checkbox_, SIGNAL(stateChanged(int )), this, SLOT(setDirection( int )));

  // Initialize the slider values.
  height_slider_->setValue( terrain_x_ );
  width_slider_->setValue( terrain_y_);

  manager_->setFixedFrame(map_frame_.c_str());
  manager_->initialize();
  manager_->startUpdate();
  update_server_ = nh_.advertiseService("/topological/edges", &MyViz::setEdges, this);
  //server_ = new boost::make_shared<actionlib::SimpleActionServer<GREdgesAction>>(nh_, "/topological/edges", boost::bind(&MyViz::execute_cb, _1), false);
  //server_->start();
  map_client_ = nh_.serviceClient<gr_map_utils::UpdateMap>("update_metric_map");

}

void MyViz::updateMapFrame(){
  std_srvs::Trigger req;
  if(update_client_.call(req)){
    ROS_INFO("Map Frame Updated");
  }
}

bool MyViz::setEdges(gr_action_msgs::GREdges2::Request& req,gr_action_msgs::GREdges2::Response& res){
 ROS_INFO("Edges received");
  terrain_y_ = req.width_meters;
  y_cells_ = default_npoints_;//ceil(terrain_y_/2);

  terrain_x_ = req.height_meters;
  nrows_ = ceil(terrain_x_/(2*robot_radius_));
  visualizeMap();
  return true;
}


MyViz::~MyViz()
{
  ROS_ERROR("D offline init");
  ROS_ERROR("D offline end");

}

void MyViz::setDirection(int state){
  std::cout << "direction State" << state << std::endl;
  direction_ =  (state==2)? 1 : -1;
  std::cout << "direction set" << direction_ << std::endl;

  visualizeMap();
}

void MyViz::setFrame(QString frame){
  map_frame_ = frame.toStdString();
}

void MyViz::setTerrainY( int value){
  terrain_y_ = value;
  y_text_->setText(std::to_string(value).c_str());
  y_cells_ = default_npoints_;//ceil(value/4);
  visualizeMap();
}

void MyViz::setRadius( int value){
  robot_radius_ = value*0.1;
  std::cout << "ROBOT Radius" << robot_radius_ << std::endl;
  nrows_ = ceil(terrain_x_/(2*robot_radius_));
  std::cout << "TerrainX " << terrain_x_ << std::endl;
  std::cout << "DEN "<<  terrain_x_/(2*robot_radius_) << std::endl;
  std::cout << "NRows " << nrows_ << std::endl;
  radius_text_->setText(std::to_string(robot_radius_).c_str());
  visualizeMap();
}

void MyViz::setTerrainX( int value ){
  terrain_x_ = value;
  x_text_->setText(std::to_string(value).c_str());
  nrows_ = ceil(terrain_x_/(2*robot_radius_));
  std::cout << "NRows " << nrows_ << std::endl;
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

void MyViz::updateAfterLoad() {
  std::cout << "UPDate AfterLoad off line " << std::endl;
  radius_text_->setText(std::to_string(robot_radius_).c_str());
  radius_slider_->setValue( ceil(robot_radius_*10) );

  height_slider_->setValue( int(terrain_x_) );
  width_slider_->setValue( int(terrain_y_) );
  angle_slider_->setValue( int(angle_*180/M_PI) );

  angle_text_->setText(std::to_string(angle_).c_str());
  x_text_->setText(std::to_string(terrain_x_).c_str());
  y_text_->setText(std::to_string(terrain_y_).c_str());
}

void MyViz::saveMap(){
  ROS_INFO("saveMap");
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


  ROS_ERROR("OK saving");
  TopoMapCollection mongo_coll = mongo_connection_.openCollection<navigation_msgs::TopologicalMap>("my_db", "maps");
  mongo_coll.insert(topo_map, makeMetadata(mongo_coll, topo_map, "wish_map4"));


  /*
  std::vector<boost::shared_ptr<navigation_msgs::TopologicalMap > >aaa;
  if(message_store_->queryNamed<navigation_msgs::TopologicalMap>(map_id,aaa)) {
    message_store_->updateNamed(map_id, topo_map);
    std::cout<<"Map  \""<<map_id<<"\" updated"<<std::endl;
  }
  else{
    storing_id_ = message_store_->insertNamed(map_id, topo_map);
    std::cout<<"Map \""<<map_id<<"\" inserted with id "<<storing_id_<<std::endl;
  }
  */
  std::ofstream out("/tmp/lastmap_id.txt");
  out << storing_id_;
  out.close();
  //
  id_maxnumberrows_ = (2*terrain_x_/robot_radius_)-1;
  std::cout << "Max Rows Idx "<< id_maxnumberrows_ << std::endl;
}
