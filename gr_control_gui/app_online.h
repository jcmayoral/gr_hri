#ifndef APPONLINE_H
#define APPONLINE_H

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
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace gr_control_gui{
  typedef std::map<std::string, geometry_msgs::Pose> NodeMap;
  typedef std::pair<std::string, std::string> Edges;
  typedef actionlib::SimpleActionClient<gr_action_msgs::GRNavigationAction> MyClient;

  class AppOnline: public MyCommonViz{
    Q_OBJECT
    public:
      AppOnline(QWidget* parent = 0 );
      virtual ~AppOnline();
    private Q_SLOTS:
      void setDesiredRow(int row);
      void executeTopoMap();
      void stopExecution();

      void setFrame(QString frame);

      void timetogoCB(const std_msgs::Float32ConstPtr time2go);
      bool executeCycle(int cycle);
      void startExecution();
      void resetHumanIntervention();

      void visualizeRowMap(int row, int& start_node, int& goal_node);
      void setMode(QListWidgetItem* item);
      void setTask(QListWidgetItem* item);
      void setResume(int span);

      void setNViaPoints(int nvia);
      void setSpan(int span);
      void updateAfterLoad();

      void feedbackCb(const gr_action_msgs::GRNavigationFeedbackConstPtr& feedback);
      bool executeRun(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res );

    private:
      int current_row_;
      visualization_msgs::MarkerArray online_marker_array_;

      ros::Publisher online_map_publisher_;
      ros::Publisher start_publisher_;
      ros::Publisher stop_publisher_;
      //ros::ServiceClient update_client_;
      MyClient gr_action_client_;
      ros::ServiceClient gr_human_intervention_client_; 

      QLabel* row_exec;
      //ros::Subscriber time_to_go_sub_;

      std::thread* t1;
      int nviapoints_;
      int span_;
      int mode_;
      std::string task_;
      QListWidget* mode_selector_;
      QListWidget* task_selector_;

      QCheckBox *checkbox_;
      QSpinBox* span_spinbox_;
      bool resume_;
      navigation_msgs::TopoExecutionStatus execution_status_;
      bool cancel_goal_;

      QLabel* status_;
      std::mutex mtx;

      //Resume execution
      int last_know_completed_row_id_;
      bool resume_execution_;

      //Execute remotely
      ros::ServiceServer remote_exec_server_;
      std::atomic<bool> isexecuting_;


  };
};
#endif
