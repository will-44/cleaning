#!/usr/bin/env python3

import rospy
import tf
import tf2_geometry_msgs
import tf2_ros
from ros_numpy import numpify
from scipy.spatial.transform import Rotation as R


class Positionning:
    def __init__(self):
        self.artag = rospy.get_param("ar_tag")
        # TF reader
        self.tf_buffer = tf2_ros.Buffer()
        # Start the TF listen and store to buffer (10s buffer)
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        return

    def get_artag_position(self):
        # get artag from world
        # try:
        #     trans_machine2artag = self.tf_buffer.lookup_transform(self.artag, "machine", rospy.Time(),
        #                                                           rospy.Duration(1.0))
        # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.loginfo("pb dans la transformation")
        # mat_surface2artag = numpify(trans_machine2artag.transform)
        try:
            trans_artag2base = self.tf_buffer.lookup_transform("base_0", "machine", rospy.Time(),
                                                                  rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
            return
        mat_machine2base = numpify(trans_artag2base.transform)
        rot = R.from_matrix(mat_machine2base[:3, :3])
        angle = rot.as_euler('xyz', degrees=False)
        rospy.loginfo("x: %f, y: %f, theta: %f", mat_machine2base[0][3], mat_machine2base[1][3], angle[2])
        return

if __name__ == '__main__':

    rospy.init_node('positionning', anonymous=True)
    pos = Positionning()
    while not rospy.is_shutdown():
        rospy.sleep(0.2)
        pos.get_artag_position()









































































