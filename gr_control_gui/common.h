#ifndef MYCOMMONVIZ_H
#define MYCOMMONVIZ_H

#include <vector>

#include <QWidget>
#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QPushButton>

#include <QGridLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include <nodes_visualizer.hpp>

#include <boost/foreach.hpp>
#include <thread>

#include <gr_map_utils/UpdateMap.h>

#include <mongodb_store/message_store.h>
#include <visualization_msgs/MarkerArray.h>
#include <navigation_msgs/TopologicalMap.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace gr_control_gui{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;

  class MyCommonViz: public QWidget{
    Q_OBJECT
    public:
      MyCommonViz( QWidget* parent = 0 );
      virtual ~MyCommonViz();

      void loadGUI();
      void visualizeMap();
      void publishRegion();

      private Q_SLOTS:

      void loadMap();
      
    protected:
      NodeMap node_map_;
      std::string storing_id_;
      MapGenerator* map_utils_;
      QGridLayout* controls_layout_;
      QVBoxLayout* main_layout_;
      rviz::RenderPanel* render_panel_;
     	mongodb_store::MessageStoreProxy* message_store_;
      navigation_msgs::TopologicalMap load_map_;
      visualization_msgs::MarkerArray marker_array_;
      std::vector<Edges> edges_;
      std::string map_frame_;

      ros::Publisher map_publisher_;
      ros::Publisher region_publisher_;
      ros::NodeHandle nh_;
      rviz::VisualizationManager* manager_;

      //TODO Update TopoInfo
      double robot_radius_;
      float terrain_y_;
      float terrain_x_;
      int y_cells_;

      //TODO THIS ARE SAME
      int x_cells_;
      int id_maxnumberrows_;

      //map rotation
      float angle_;
  };
};
#endif 
