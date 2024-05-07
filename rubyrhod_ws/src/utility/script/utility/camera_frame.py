#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

def image_callback(msg):
    # Create a copy of the received message
    modified_msg = msg

    # Modify the frame_id in the header
    modified_msg.header.frame_id = "camera_base"

    # Publish the modified message to a new topic
    pub.publish(modified_msg)

def image_frame_changer():
    rospy.init_node('image_frame_changer', anonymous=True)

    # Subscribe to the /rgb topic
    rospy.Subscriber('/rgb/image_raw', Image, image_callback)

    # Advertise a new topic for the modified messages
    global pub
    pub = rospy.Publisher('/modified_rgb', Image, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        image_frame_changer()
    except rospy.ROSInterruptException:
        pass
