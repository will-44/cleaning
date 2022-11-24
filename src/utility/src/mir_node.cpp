//
// Created by sycobot on 06/05/22.
//


#include "../includes/mir.h"
#include <geometry_msgs/Pose2D.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

/*
 * This node send the position ask by the user to the mir action
 * param: geometry_msgs::Pose2D, x,y,theta
 */

void callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  Mir mir("mir");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Bool>("mir_go_to_result", 1000);

  geometry_msgs::Pose2D pose;
  pose.x = msg->x;
  pose.y = msg->y;
  pose.theta = msg->theta;

  bool result = mir.navMir(pose);

  std_msgs::Bool answer;
  answer.data = result;
  pub.publish(answer);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mir");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("mir_go_to", 1000, callback);
  ros::spin();

  return 0;
}
