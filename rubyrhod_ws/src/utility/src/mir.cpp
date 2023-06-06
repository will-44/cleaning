//
// Created by sycobot on 25/04/22.
//

#include "../includes/mir.h"
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

/**
 * @brief This class call the action to move the mir base
 * @param name The name is needed to identify the action (or not ?)
 */
Mir::Mir(const std::string &name) : client_base("move_base", true)
{
    client_base.waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

/**
 * @brief This methode give the position during the movement
 * @return pose 2D
 */
geometry_msgs::Pose2D Mir::getPose(){
    return mir_pose;
}

/**
 * @brief this method is called when the action is done
 * @param state the state is succeed or aborted
 * @param result there is not result for this action
 */
void Mir::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr & result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * @brief This method is called when the action is call
 */
void Mir::activeCb()
{
    ROS_INFO("coord x: , y: ");
}

/**
 * @brief This method is called during the execution
 * @param feedback It give the actual position of the mir
 */
void Mir::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("coord x: %f, y: %f,  theta: %f", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y, feedback->base_position.pose.orientation.z);
    mir_pose.x = feedback->base_position.pose.position.x;
    mir_pose.y = feedback->base_position.pose.position.y;
    mir_pose.theta = feedback->base_position.pose.orientation.z;

}

/**
 * @brief The user calls this method to start the action
 * @param pose The desired position x, y, theta
 * @return  the result of the action true or false
 */
bool Mir::navMir(const geometry_msgs::Pose2D &pose)
{
    ROS_INFO("I heard: [%f]", pose.x);
    //tell the action client that we want to spin a thread by default


    //wait for the action server to come up
    while (!client_base.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    //     create quaternion
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0,  pose.theta);


    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pose.x;
    goal.target_pose.pose.position.y = pose.y;
    goal.target_pose.pose.orientation.z = myQuaternion.getZ();
    goal.target_pose.pose.orientation.w = myQuaternion.getW();
    ROS_INFO("I heard: [%f] et W: %f", myQuaternion.getZ(), myQuaternion.getW());
    ROS_INFO("Sending goal");

    client_base.sendGoal(goal,
                         boost::bind(&Mir::doneCb, this, _1, _2),
                         boost::bind(&Mir::activeCb, this),
                         boost::bind(&Mir::feedbackCb, this, _1));
    client_base.waitForResult();

    if (client_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        ROS_INFO("Hooray, the base moved 1 meter forward");
        return true;
    }
    else {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        return false;
    }

}

