#!/usr/bin/env python3

import rospy
from doosan import Doosan
import rospkg
import tf2_ros
from ros_numpy import numpify, msgify
import numpy as np 
from geometry_msgs.msg import Transform, TransformStamped

if __name__ == '__main__':
    # open the doosan class to set the colissions
    rospy.init_node('add_basic_colision', anonymous=True)
    rospy.loginfo("trans_world" )
    # arm = Doosan()
    # arm.add_machine_colision(rospkg.RosPack().get_path('utility') + "/mesh/scie_3.obj")
    # rospy.sleep(5)
    
    broadcaster_artag2machine = tf2_ros.StaticTransformBroadcaster()
    broadcaster_map2machine = tf2_ros.StaticTransformBroadcaster()
    # Defined when the artag was placed
    mat_artag2machine = [[0.0445165, -0.9988334, -0.0187112,  -0.8851627177186572],
                        [0.0373603,  -0.0170522, 0.9991564, -0.1324471788338875],
                        [-0.9983098, -0.0451780, 0.0365576, -0.956239983059406],
                        [0,          0,          0,         1]]
    mat_artag2machine = np.asarray(mat_artag2machine)
    trans_artag2machine =  msgify(Transform, mat_artag2machine)
    print(trans_artag2machine)
        # send tf
    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "ar_marker_6"
    static_transform_stamped.child_frame_id = "machine"
    static_transform_stamped.transform = trans_artag2machine

    broadcaster_artag2machine.sendTransform(static_transform_stamped)


    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    try:
        trans_surface2artag = tf_buffer.lookup_transform('surface', "ar_marker_6", rospy.Time(),
                                                rospy.Duration(1.0))
    except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation")
    mat_surface2artag = numpify(trans_surface2artag.transform)

    rospy.loginfo(mat_surface2artag)

    try:
        trans_map2surface = tf_buffer.lookup_transform('map', "surface", rospy.Time(),
                                                rospy.Duration(1.0))
    except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("pb dans la transformation")
    mat_map2surface = numpify(trans_map2surface.transform)

    rospy.loginfo(mat_map2surface)
    # print("calcul:")

    mat_map2machine = np.dot(mat_map2surface, np.dot(mat_surface2artag, mat_artag2machine))

    trans_map2machine =  msgify(Transform, mat_map2machine)
    # print(trans_map2machine)
    # send tf
    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "machine"
    static_transform_stamped.transform = trans_map2machine
    broadcaster_map2machine.sendTransform(static_transform_stamped)
    arm = Doosan()
    arm.add_machine_colision(rospkg.RosPack().get_path('utility') + "/mesh/scie_3.obj")
    while(not rospy.is_shutdown()):
        static_transform_stamped.header.stamp = rospy.Time.now()
        broadcaster_map2machine.sendTransform(static_transform_stamped)
        rospy.sleep(0.01)



    # try:
    #     trans_world = tf_buffer.lookup_transform('map', "surface", rospy.Time(),
    #                                             rospy.Duration(1.0))
    # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rospy.loginfo("pb dans la transformation")
    # rospy.loginfo(numpify(trans_world.transform))




    # arm.go_home()
    