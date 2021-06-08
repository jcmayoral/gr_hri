#ifndef MYOFFLINEVIZ_H
#define MYOFFLINEVIZ_H

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
#include <QTextEdit>
#include <QCheckBox>


#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>

#include <nodes_visualizer.hpp>

#include <ros/ros.h>

#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>
#include <thread>


#include <gr_map_utils/UpdateMap.h>

#include <common.h>

#include <gr_action_msgs/GREdgesAction.h>
#include <gr_action_msgs/GREdges2.h>

#include <actionlib/server/simple_action_server.h>


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
      void setAngle( int value );
      void setRadius( int value );

      void setDirection(int state);
      void saveMap();
      void deleteTopoMap();
      void setFrame(QString frame);
      bool setEdges(gr_action_msgs::GREdges2::Request& req,gr_action_msgs::GREdges2::Response& res);
      //void execute_cb(const GREdgesActionGoal& goal);
      void updateAfterLoad();
    
    private:
      //boost::shared_ptr<actionlib::SimpleActionServer<GREdgesAction>> server_;
      int current_row_;
      ros::ServiceServer update_server_;

      int id_maxnumberrows_;
      QTextEdit* angle_text_;
      QTextEdit* x_text_;
      QTextEdit* y_text_;
      QTextEdit* radius_text_;

      //Direction
      QCheckBox *checkbox_;

      QSlider* height_slider_;
      QSlider* angle_slider_;
      QSlider* width_slider_;
      QSlider* radius_slider_;

  };
};
  // END_TUTORIAL
#endif // MYOFFLINEVIZ_H
