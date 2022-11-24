// Bring in my package's API, which is what I'm testing

// Bring in gtest.
#include "../includes/mir.h"
#include <stdlib.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"


// Declare a test
/**
 * @brief This test for a valid position
 */
TEST(TestSuite, testCase1)
{
  Mir mir("mir");
  geometry_msgs::Pose2D pose;
  pose.x = 25;
  pose.y = 12;
  pose.theta = 3.1415/2;
  bool result = mir.navMir(pose);
  EXPECT_TRUE(result);
  EXPECT_TRUE(abs(mir.getPose().x -pose.x) <= 0.1);

}

/**
 * @brief This test for position out of the map
 */
TEST(TestSuite, testCase2)
{
  Mir mir("mir");
  geometry_msgs::Pose2D pose;
  pose.x = 2002.27;
  pose.y = 900.77;
  pose.theta = 3.1415/2;
  bool result = mir.navMir(pose);
  EXPECT_FALSE(result);

}

/**
 * @brief This tet is for a position occupied
 */
TEST(TestSuite, testCase3)
{
  Mir mir("mir");
  geometry_msgs::Pose2D pose;
  pose.x = 17.00;
  pose.y = 12.00;
  pose.theta = 3.1415/2;
  bool result = mir.navMir(pose);
  EXPECT_FALSE(result);

}

/**
 * @brief This test is for a second position faraway
 */
TEST(TestSuite, testCase4)
{
  Mir mir("mir");
  geometry_msgs::Pose2D pose;
  pose.x = 19.50;
  pose.y = 11.80;
  pose.theta = 3.1415;
  bool result = mir.navMir(pose);
  EXPECT_TRUE(result);
  EXPECT_TRUE(abs(mir.getPose().x -pose.x) <= 0.1);
}





// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  nav_msgs::Odometry check;
  try
  {
    check  = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odom",ros::Duration(65)));
  }
  catch(ros::Exception &e)
  {
    ROS_WARN("Timeout we did not connected to the MiR, the test won't work");
  }
  return RUN_ALL_TESTS();
}