#!/usr/bin/env python3

import rospy
from doosan import Doosan
from robotiq import Robotiq
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import rviz_tool
import tf

def callback(data):
    '''

    :param data: The position of the object geometry_msgs.msg.Pose
    :return: Send a message on the topics grasp_result. True if the action is done or False if some issue append
    '''
    result = False
    arm = Doosan()
    gripper = Robotiq()
    pub = rospy.Publisher('grasp_result', Bool, queue_size=10)
    gripper.open()

    while(gripper.status().gPR != 0):
        rospy.sleep(1)

    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]


    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    data.pose.orientation.x = quaternion[0]
    data.pose.orientation.y = quaternion[1]
    data.pose.orientation.z = quaternion[2]
    data.pose.orientation.w = quaternion[3]



    rviz_tool.display_marker(Marker.ARROW, data.pose.position.x,
                             data.pose.position.y, data.pose.position.z,
                             data.pose.orientation.x, data.pose.orientation.y,
                             data.pose.orientation.z, data.pose.orientation.w, data.header.frame_id)
    
    result = arm.go_to_l(data)
    while((arm.check_motion() != 0)):
        rospy.sleep(1)

    gripper.close()

    pub.publish(result)

if __name__ == '__main__':
    rospy.init_node('grasp_node', anonymous=True)
    rospy.Subscriber("/dsr01m1013/grasp", PoseStamped, callback)

    # just activate the gripper
    gripper = Robotiq()
    gripper.reset()
    rospy.sleep(2)
    gripper.activate()
    # rospy.sleep(5)
    print("activate")
    rospy.spin()