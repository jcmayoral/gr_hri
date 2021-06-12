#ifndef MYONLINEVIZ_H
#define MYONLINEVIZ_H

#include <nodes_visualizer.hpp>

#include <ros/ros.h>

#include <std_msgs/Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>

#include <common.h>

#include<move_base_msgs/MoveBaseActionGoal.h>
#include<gr_action_msgs/GRNavigationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigation_msgs/TopoExecutionStatus.h>

#include <QCheckBox>

#include <std_srvs/Trigger.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}


namespace gr_control_gui{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;
  typedef actionlib::SimpleActionClient<gr_action_msgs::GRNavigationAction> MyClient;


  class MyViz: public MyCommonViz{
    Q_OBJECT
    public:
      MyViz( QWidget* parent = 0 );
      virtual ~MyViz();

      private Q_SLOTS:
      void setDesiredRow(int row);
      void executeTopoMap();
      void stopExecution();

      void setFrame(QString frame);

      void timetogoCB(const std_msgs::Float32ConstPtr time2go);
      void executeCycle(int cycle);
      void visualizeRowMap(int row);
      void setMode(QListWidgetItem* item);
      void setNViaPoints(int nvia);
      void setSpan(int span);
      void updateAfterLoad();

      void feedbackCb(const gr_action_msgs::GRNavigationFeedbackConstPtr& feedback);

    private:
      int current_row_;
      visualization_msgs::MarkerArray online_marker_array_;

      ros::Publisher online_map_publisher_;
      ros::Publisher reset_publisher_;
      ros::ServiceClient update_client_;
      MyClient gr_action_client_;

      QLabel* time_to_go;
      ros::Subscriber time_to_go_sub_;

      std::thread* t1;
      int nviapoints_;
      int span_;
      int mode_;
      QListWidget* mode_selector_;
      QCheckBox *checkbox_;
      QSpinBox* span_spinbox_;
      bool resume_;
      navigation_msgs::TopoExecutionStatus execution_status_;
      bool cancel_goal_;
  };
};
  // END_TUTORIAL
#endif // MYONLINEVIZ_H
