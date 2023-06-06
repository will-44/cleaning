#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "rospy_tutorials/Floats.h"
#include "std_msgs/Bool.h"
using namespace cv;
using namespace std;

ros::Publisher pcl_pub;
ros::Publisher camera_info_pub;
Mat mask_tool;
vector<Point2f> dust_poses;
sensor_msgs::CameraInfo camera_info;
int radius = 200;
cv_bridge::CvImagePtr cv_ptr;
bool toggle_octomap = false;
int iter = 0;


void callback_dust(const rospy_tutorials::Floats::ConstPtr& msg)
/**
 *
 * @param msg The list of dust pixel positions
 */
{
    vector<float> data = msg->data;
    dust_poses.resize(data.size() / 2);
    for (size_t i = 0; i < data.size() / 2; i++)
    {
        dust_poses[i].x = data[2 * i];
        dust_poses[i].y = data[2 * i + 1];
    }
}

void callback_octomap(const std_msgs::Bool::ConstPtr& msg)
/**
 * Toogle the the send of the depth map to the octomap
 * @param msg boolean
 */
{
    toggle_octomap = msg->data;
}

void callback_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
/**
 * Recieve the camera info
 * @param msg camera info
 */
{
    camera_info = *msg;
}

void callback_depth(const sensor_msgs::Image::ConstPtr& msg)
/**
 * Receive the depth map
 * @param msg
 */
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat cv_image = cv_ptr->image;

    // Remove the tool from depth map
    cv_image.setTo(Scalar(0), mask_tool);

    // Remove the dust spots
    Mat mask_dust(cv_image.size(), CV_8UC1, Scalar(0));
    for (Point2f dust : dust_poses)
    {
        circle(mask_dust, dust, radius, Scalar(255), -1);
    }
    cv_image.setTo(Scalar(0), mask_dust);

    // Convert the image back to ROS format and publish
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id = "rgb_camera_link";
    out_msg.header.stamp = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    out_msg.image = cv_image;
    if(toggle_octomap && iter >= 10){
        iter = 0;
        pcl_pub.publish(out_msg.toImageMsg());
    }
    iter += 1;

    // Publish camera info
    camera_info.header.stamp = out_msg.header.stamp;
    camera_info_pub.publish(camera_info);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_limit");
    ros::NodeHandle nh;
    std::string path_mask = ros::package::getPath("cleaning") + "/img/mask_depth.png";

    mask_tool = imread(path_mask, cv::IMREAD_GRAYSCALE);

    // Setup publishers and subscribers
    pcl_pub = nh.advertise<sensor_msgs::Image>("/depth_filter/image_raw", 10);
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/depth_filter/camera_info", 10);

    ros::Subscriber sub_image = nh.subscribe("/depth_to_rgb/image_raw", 10, callback_depth);
    ros::Subscriber sub_dust = nh.subscribe("/dust_pixels", 1, callback_dust);
    ros::Subscriber sub_info = nh.subscribe("/depth_to_rgb/camera_info", 1, callback_info);
    ros::Subscriber sub_activate = nh.subscribe("/toggle_octomap", 1, callback_octomap);


    ros::spin();
}