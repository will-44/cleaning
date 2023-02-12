#!/usr/bin/env python3

from geometry_msgs.msg import Pose2D
from math import sqrt
from math import atan2


def get_pose_for_mobile(mir_pose, object_pose):
    '''
    This methode get the actual mir pose and the object pose and determine
    the best position to send the mir.
    :param mir_pose: the position of the mir Pose2D (euler angle)
    :param object_pose: the pose of the object Pose2D (euler angle)
    :return: the pose to go for the mir
    '''
    # There is two point crossing the reach circle. The first one is A and the second is B
    pose_a = Pose2D()
    pose_b = Pose2D()

    # get a and b coef from the line equation y = ax + b between the mir pose and the object pose
    a = (mir_pose.y - object_pose.y) / (mir_pose.x - object_pose.x)
    b = mir_pose.y - (a * mir_pose.x)

    # reachability of the doosan
    reach = 0.8  # metre
    # Eq 2nd deg coef: Ax^2 + Bx + C = 0
    # From the intersection between the line and the circle center on the objet and radius reach
    A = 1 + a**2
    B = -2 * object_pose.x + 2 * (b - object_pose.y) * a
    C = (b-object_pose.y)**2 + object_pose.x**2 - reach**2
    delta = B**2 - 4*A*C
    #  The intersection give an 2de degree equation that we resolve to find two points
    pose_a.x = (-B - sqrt(delta)) / (2 * A)
    pose_b.x = (-B + sqrt(delta)) / (2 * A)

    pose_a.y = a * pose_a.x + b
    pose_b.y = a * pose_b.x + b


    # keep the closest point from the robot (abs norme smallest)
    norme_a = sqrt((pose_a.x - mir_pose.x) ** 2 + (pose_a.y - mir_pose.y) ** 2)
    norme_b = sqrt((pose_b.x - mir_pose.x) ** 2 + (pose_b.y - mir_pose.y) ** 2)

    result = pose_a
    if norme_b >= norme_a:
        result = pose_a
    else:
        result = pose_b

    # Get the theta angle to point the mir in the direction of the object
    result.theta = atan2((object_pose.y - result.y), (object_pose.x - result.x))

    return result
