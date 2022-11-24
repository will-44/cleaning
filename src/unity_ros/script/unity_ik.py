#!/usr/bin/env python3

import rospy

from utility.doosan import Doosan
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Global publisher to create once
pub = rospy.Publisher('/unity/ik/returned_pos', JointState, queue_size=1)

def ik_solver(pose, header):
    '''
    This method will do the inverse kinematic and publish the output in a JointStateMsg
    :param pose: The desired position in meter and in radian in a vector6
    :param header: the header to initialise the header of the JointState that woll be publish
    '''    
    
    # Initialize the message
    joint_position = JointState()
    joint_position.header = header
    joint_configuration = arm.ikin(pose, 0, 0)
    joint_position.position = joint_configuration.conv_posj
    
    # Publish the result of the inverse kinematics
    pub.publish(joint_position)

def transform_coordinate_system(quaternion):
    '''
    Transfert the rotation of an object from it's quaternion to a zyz euler coordinate system
    :param quaternion: The rotation in quaternion in a xyz coordinate system - in quaternion
    :return: The new rotation in the euler zyz coordinate system - in radian
    '''
    
    # Get the matrix 3x3 for the rotation from the quaternion
    matrix = R.from_quat(quaternion)
    
    # Change the coordinate system
    return matrix.as_euler('zyz')    

def callback(msg):
    '''
    Create a position variable that will be use as an input for the ik_solver
        with the message received
    :param msg: The desired position to do the IK, geometry_msgs.PoseStamped
    :return:
    '''
    
    # Change quaternion into radian
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w)
    
    # Get the euler angle for the rotation
    euler_zyz = transform_coordinate_system(quaternion)
    
    # Set the position in a vector to be use as an input for the IK method
    pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            euler_zyz[0], euler_zyz[1], euler_zyz[2]]
    
    try:
        ik_solver(pose, msg.header)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error in the inverse kinematics calcul")

if  __name__ == "__main__":
    global arm
    rospy.init_node('unity_ik_node', anonymous=True)
    rospy.Subscriber('/unity/ik/desired_pos', PoseStamped, callback, queue_size=1)
    arm = Doosan()

    # Wait to let the IK resolve
    rospy.sleep(.5)
    rospy.spin()