#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np

vells = np.asarray([0, 0, 0, 0, 0, 0])
accels = np.asarray([0, 0, 0, 0, 0, 0])

best_vel = [0, 0, 0, 0, 0, 0]

def callback(msg):
    """
    This fct get the jointState msg and calculate the acceleration of each joint
    :param msg: JointState message
    :return: print the speed and acceleration of each joints
    """
    #     Save the last jointState msg and compare it with the new one to get the velovity and acceleration
    # print(msg)
    global last_msg
    if last_msg is None:
        last_msg = msg
        return
    else:
        # calculate the time between the two msg
        dt = msg.header.stamp.to_sec() - last_msg.header.stamp.to_sec()
        # calculat the velocity from the position
        vel = [(msg.position[i] - last_msg.position[i]) / dt for i in range(len(msg.position))]
        # save the last velocity
        msg.velocity = vel
        # calculate the acceleration
        acc = [(msg.velocity[i] - last_msg.velocity[i]) / dt for i in range(len(msg.velocity))]

        # print the best vel and accel for all joints
        np.append(vells, vel)
        np.append(accels, acc)

        # if the best velocity from vells change , print it
        if np.max(vells, axis=0) != np.max(vel, axis=0):
            # print("best vel: ", np.max(vel))
            # keep the best velocity for each joint to best_vel
            for i in range(len(vel)):
                if vel[i] > best_vel[i]:
                    best_vel[i] = vel[i]
                    print("best vel for each joint: ", best_vel)

        # if the best acceleration from accels change , print it
        if np.max(accels, axis=0) != np.max(acc, axis=0):
            # print("best acc: ", np.max(acc))
            # print("best acc for each joint: ", np.max(accels, axis=0))
            for i in range(len(acc)):
                if acc[i] > best_vel[i]:
                    best_vel[i] = acc[i]
                    print("best acc for each joint: ", best_vel)






        # print the acceleration and velocity
        # print("acc: ", acc)
        # print("vel: ", msg.velocity)

        # save the last velocity
        last_msg.velocity = vel

        # save the last msg
        last_msg = msg
        # print the max


if __name__ == '__main__':
    rospy.init_node("calculate_accel")
    print("coucou")
    global last_msg
    last_msg = None
    rospy.Subscriber("/dsr01m1013/joint_states", JointState, callback)
    rospy.spin()
