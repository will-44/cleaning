//
// Created by sycobot on 25/04/22.
//

#ifndef SRC_MIR_H
#define SRC_MIR_H



#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose2D.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;



class Mir
{
private:
  ros::NodeHandle nh;
  moveBaseClient client_base;
  std::string action_name;
  move_base_msgs::MoveBaseFeedback feedback;
  move_base_msgs::MoveBaseResult result;
  geometry_msgs::Pose2D mir_pose;


public:
  Mir(const std::string &name);
  geometry_msgs::Pose2D getPose();
  bool navMir(const geometry_msgs::Pose2D &pose);
private:
  void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCb();
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

};

#endif//SRC_MIR_H
