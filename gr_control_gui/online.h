#ifndef MYONLINEVIZ_H
#define MYONLINEVIZ_H

#include <nodes_visualizer.hpp>

#include <ros/ros.h>

#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>

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
      void setDesiredRow(int row);
      void executeTopoMap();
      void visualizeMap();
      void saveMap();
      void deleteTopoMap();
      void setFrame(QString frame);

      void publishRegion();
      void timetogoCB(const std_msgs::Float32ConstPtr time2go);
      void executeCycle(int cycle);
      bool existsMap();

    private:
      rviz::VisualizationManager* manager_;
      rviz::RenderPanel* render_panel_;
      MapGenerator* map_utils_;
      int x_cells_;
      int y_cells_;
      int current_row_;
      ros::NodeHandle nh_;
      ros::Publisher map_publisher_;
      ros::Publisher reset_publisher_;
      ros::Publisher region_publisher_;
      ros::ServiceClient update_client_;

      double robot_radius_;
      float terrain_y_;
      float terrain_x_;
      visualization_msgs::MarkerArray marker_array_;
     	mongodb_store::MessageStoreProxy* message_store_;

      NodeMap node_map_;
      std::vector<Edges> edges_;

      std::string storing_id_;

      QLabel* time_to_go;
      ros::Subscriber time_to_go_sub_;
      int id_maxnumberrows_;

      std::thread* t1;

      std::string map_frame_;
  };
};
  // END_TUTORIAL
#endif // MYONLINEVIZ_H
