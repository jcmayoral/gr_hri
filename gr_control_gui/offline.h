#ifndef MYVIZ_H
#define MYVIZ_H

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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <navigation_msgs/TopologicalMap.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>
#include <thread>

#include <gr_map_utils/UpdateMap.h>

#include <common.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace gr_control_gui{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;

  class MyViz: public MyCommonViz{
    Q_OBJECT
    public:
      MyViz( QWidget* parent = 0 );
      virtual ~MyViz();

      private Q_SLOTS:
      void setTerrainY( int value);
      void setTerrainX( int value);
      void visualizeMap();
      void saveMap();
      void deleteTopoMap();
      void setFrame(QString frame);

      void publishRegion();

    private:
      int x_cells_;
      int y_cells_;
      int current_row_;
      ros::ServiceClient update_client_;

      double robot_radius_;
      float terrain_y_;
      float terrain_x_;
      visualization_msgs::MarkerArray marker_array_;

      NodeMap node_map_;
      std::vector<Edges> edges_;

      std::string storing_id_;

      int id_maxnumberrows_;

      std::string map_frame_;
  };
};
  // END_TUTORIAL
#endif // MYVIZ_H
