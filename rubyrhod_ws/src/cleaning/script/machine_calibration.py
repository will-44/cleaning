#!/usr/bin/env python3

import sys

import numpy as np
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros
from utility.doosan import Doosan
import yaml
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped
from ros_numpy import numpify, msgify


class MachineCalibration:
    def __init__(self):

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.broadcaster_artag2machine = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster_map2machine = tf2_ros.StaticTransformBroadcaster()
        self.artag = rospy.get_param("/ar_tag")

    def set_mesh_pos(self):
        arm = Doosan()
        collision_stamp = PoseStamped()
        collision_stamp.header.frame_id = "world"
        collision_stamp.pose.orientation.x = 0
        collision_stamp.pose.orientation.y = 0
        collision_stamp.pose.orientation.z = 0.707
        collision_stamp.pose.orientation.w = 0.707
        collision_stamp.pose.position.x = 1.83+0.3
        collision_stamp.pose.position.y = 0.2#0.68-0.2
        collision_stamp.pose.position.z = -0.76+0.04

        arm.add_machine_colision(rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"), collision_stamp)

    def get_machine2ar(self):
        """
        This method use the tf from the robot to the machine to calcul the tf from the machine to the arTag place on it
        The initial tf is needed.
        :return:
        """
        arm = Doosan()
        try:
            trans_surface2artag = self.tf_buffer.lookup_transform(self.artag, "machine", rospy.Time(),
                                                                  rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        mat_surface2artag = numpify(trans_surface2artag.transform)
        rospy.loginfo(mat_surface2artag)

        try:
            trans_world = self.tf_buffer.lookup_transform('world', "machine", rospy.Time(),
                                                          rospy.Duration(1.0))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")

        collision_stamp = PoseStamped()
        collision_stamp.header.frame_id = "machine"
        collision_stamp.pose.orientation.w = 1
        collision_stamp.pose.orientation.y = 0

        collision_stamp = tf2_geometry_msgs.do_transform_pose(collision_stamp, trans_world)

        arm.add_machine_colision(rospkg.RosPack().get_path('cleaning') + rospy.get_param("/machine_mesh"), collision_stamp)

        path_config = rospkg.RosPack().get_path('cleaning') + "/config/config.yaml"

        mat_name = rospy.get_param("/machine2ar_calibration")
        with open(path_config, 'r') as file:
            # Charger le contenu du fichier YAML dans un dictionnaire
            config_param = yaml.safe_load(file)
        print(config_param)
        config_param[mat_name] = mat_surface2artag.tolist()
        print(config_param)


        # Écrire les données mises à jour dans le fichier YAML
        with open(path_config, 'w') as file:
            yaml.dump(config_param, file)
        rospy.set_param("/" + mat_name, mat_surface2artag.tolist())

    def pub_machine_tf(self):
        """
        This method publish the machine tf in the map frame from the arTag position.
        :return:
        """
        # Defined when  the artag was placed
        mat_name = "/" + rospy.get_param("/machine2ar_calibration")

        mat_artag2machine = np.asarray(rospy.get_param(mat_name))

        trans_artag2machine = msgify(Transform, mat_artag2machine)
        print(trans_artag2machine)
        # send tf
        static_transform_stamped_ar2mach = TransformStamped()
        static_transform_stamped_ar2mach.header.stamp = rospy.Time.now()
        static_transform_stamped_ar2mach.header.frame_id = self.artag
        static_transform_stamped_ar2mach.child_frame_id = "machine"
        static_transform_stamped_ar2mach.transform = trans_artag2machine

        self.broadcaster_artag2machine.sendTransform(static_transform_stamped_ar2mach)

        try:
            trans_surface2artag = self.tf_buffer.lookup_transform('world', self.artag, rospy.Time(),
                                                                  rospy.Duration(90))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("pb dans la transformation")
        mat_surface2artag = numpify(trans_surface2artag.transform)

        # try:
        #     trans_map2surface = self.tf_buffer.lookup_transform('map', "surface", rospy.Time(),
        #                                                         rospy.Duration(90))
        # except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.loginfo("pb dans la transformation")
        # mat_map2surface = numpify(trans_map2surface.transform)

        mat_map2machine = np.dot(mat_surface2artag, mat_artag2machine)

        trans_map2machine = msgify(Transform, mat_map2machine)
        # send tf
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = "machine"
        static_transform_stamped.transform = trans_map2machine
        self.broadcaster_map2machine.sendTransform(static_transform_stamped)

        while (not rospy.is_shutdown()):
            static_transform_stamped_ar2mach.header.stamp = rospy.Time.now()
            self.broadcaster_artag2machine.sendTransform(static_transform_stamped_ar2mach)
            rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('machine_calibration', anonymous=True)
    mode = sys.argv[1]
    calibrator = MachineCalibration()
    if mode == "get_transform":
        calibrator.get_machine2ar()
    elif mode == "set_mesh_pos":
        calibrator.set_mesh_pos()
    else:
        calibrator.pub_machine_tf()
