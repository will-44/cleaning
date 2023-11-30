#include <ros/ros.h>
#include <sensor_msgs/Image.h>

ros::Publisher pub;

void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    // Create a copy of the received message
    sensor_msgs::Image modified_msg = *msg;

    // Modify the frame_id in the header
    modified_msg.header.frame_id = "camera_base";

    // Publish the modified message to a new topic
    pub.publish(modified_msg);
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "image_frame_changer");
    ros::NodeHandle n;

    // Subscribe to the /rgb topic
    ros::Subscriber sub = n.subscribe("/rgb", 1, imageCallback);

    // Advertise a new topic for the modified messages
    pub = n.advertise<sensor_msgs::Image>("/modified_rgb", 1);

    // Spin to receive callbacks
    ros::spin();

    return 0;
}
