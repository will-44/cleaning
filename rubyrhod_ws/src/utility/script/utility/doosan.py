#!/usr/bin/env python3

import rospy

from dsr_msgs.srv import Ikin
from dsr_msgs.srv import Fkin
from dsr_msgs.srv import MoveJoint
from dsr_msgs.srv import MoveLine
from dsr_msgs.srv import MoveSpiral
from dsr_msgs.srv import MoveStop
from dsr_msgs.srv import MoveResume
from dsr_msgs.srv import GetCurrentPose
from dsr_msgs.srv import GetCurrentPosj
from dsr_msgs.srv import CheckMotion
from dsr_msgs.srv import GetCurrentRotm
from dsr_msgs.srv import SetCurrentTcp
from dsr_msgs.srv import SetCurrentTool
from dsr_msgs.srv import TaskComplianceCtrl
from dsr_msgs.srv import ReleaseComplianceCtrl

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotState, PositionIKRequest, AttachedCollisionObject
from geometry_msgs.msg import PoseStamped

import numpy as np
import sys
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import math

from std_srvs.srv import Empty
import xml.etree.ElementTree as ET


class Doosan:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planner_id("RRTConnect")
        self.move_group.set_planning_time(2)
        # Set the end effector as the tcp link, define in the doosan urdf
        # self.move_group.set_end_effector_link("world")

        # This sleep time is necessary to wait the init of the scene
        rospy.sleep(2)
        # if first_time:
        # Add the mir as a box
        mir_pose = geometry_msgs.msg.PoseStamped()
        mir_pose.header.frame_id = "world"  # world frame (careful with the simu to real)
        mir_pose.pose.position.x = 0
        mir_pose.pose.position.y = 0.0
        mir_pose.pose.position.z = -0.35
        mir_pose.pose.orientation.x = 0.0  # 0.0
        mir_pose.pose.orientation.y = 0.0  # 0.0
        mir_pose.pose.orientation.z = 0  # 0.0
        mir_pose.pose.orientation.w = 0  # 1.0
        mir_name = "mir"
        self.scene.attach_box('world', mir_name, mir_pose, size=(0.50, 0.40, 0.7))
        # Add the controler doosan as a box
        # controler_pose = geometry_msgs.msg.PoseStamped()
        # controler_pose.header.frame_id = "world"  # world frame (careful with the simu to real)
        # controler_pose.pose.position.x = -0.1
        # controler_pose.pose.position.y = 0.0
        # controler_pose.pose.position.z = -0.2
        # controler_pose.pose.orientation.x = 0.0  # 0.0
        # controler_pose.pose.orientation.y = 0.0  # 0.0
        # controler_pose.pose.orientation.z = 0.0  # 1.57
        # controler_pose.pose.orientation.w = 1.0  # 1.0
        # controler_name = "controler"
        # self.scene.attach_box('world', controler_name, controler_pose, size=(0.40, 0.50, 0.40))
        # lower, upper
        self.joint_limit = np.array(self.get_joint_limit())
        self.joint_index_max = self.get_joint_limit_index(np.mean(self.joint_limit, axis=1))
        # print(self.joint_index_max)
        # Joint state sub
        # self.joint_state_sub = rospy.Subscriber("/dsr01m1013/joint_states", JointState, self.callback_depth)
        # self.joint_state = 0
        # self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
        self.get_pos_x()


    # def callback_joint_state(self, msg):
    #     self.joint_state = msg
    def get_pose(self):
        return self.move_group.get_current_pose()

    def set_tcp(self, name):
        self.move_group.set_end_effector_link(name)

    def dsr_set_tcp(self, name):
        """
        Set tcp position specify in the teach pendant
        :param name: tcp name in the teach pendant
        :return:
        """

        try:
            rospy.wait_for_service('/dsr01m1013/tcp/set_current_tcp', timeout=5)
        except:
            return False
        try:
            set_tcp_srv = rospy.ServiceProxy('/dsr01m1013/tcp/set_current_tcp', SetCurrentTcp)
            result = set_tcp_srv(name)
            return True
        except rospy.ServiceException as e:
            return False
            print("Service call failed: s")

    def dsr_emergency_stop(self):
        try:
            rospy.wait_for_service('/dsr01m1013/motion/move_stop', timeout=5)
        except:
            return False
        try:
            stop_srv = rospy.ServiceProxy('/dsr01m1013/motion/move_stop', MoveStop)
            result = stop_srv(0)
            return True
        except rospy.ServiceException as e:
            return False
            print("Service call failed: s")

    def dsr_resume(self):
        try:
            rospy.wait_for_service('/dsr01m1013/motion/move_resume', timeout=5)
        except:
            return False
        try:
            resume_srv = rospy.ServiceProxy('/dsr01m1013/motion/move_resume', MoveResume)
            result = resume_srv()
            return True
        except rospy.ServiceException as e:
            return False
            print("Service call failed: s")

    def dsr_set_tool(self, name):
        """
        Set tool weight specify in the teach pendant
        :param name: tool name in the teach pendant
        :return:
        """

        try:
            rospy.wait_for_service('/dsr01m1013/tool/set_current_tool', timeout=5)
        except:
            return False
        try:
            set_tool_srv = rospy.ServiceProxy('/dsr01m1013/tool/set_current_tool', SetCurrentTool)
            result = set_tool_srv(name)
            return result
        except rospy.ServiceException as e:
            return False
            print("Service call failed: s")

    def MMDegToMRad(self, pos):
        pos[0] = pos[0] / 1000
        pos[1] = pos[1] / 1000
        pos[2] = pos[2] / 1000
        pos[3] = pos[3] * np.pi / 180
        pos[4] = pos[4] * np.pi / 180
        pos[5] = pos[5] * np.pi / 180
        return pos

    def MRadToMMDeg(self, pos):
        pos[0] = pos[0] * 1000
        pos[1] = pos[1] * 1000
        pos[2] = pos[2] * 1000
        pos[3] = pos[3] * 180 / np.pi
        pos[4] = pos[4] * 180 / np.pi
        pos[5] = pos[5] * 180 / np.pi
        return pos

    def RadToDeg(self, pos):
        pos[0] = pos[0] * 180 / np.pi
        pos[1] = pos[1] * 180 / np.pi
        pos[2] = pos[2] * 180 / np.pi
        pos[3] = pos[3] * 180 / np.pi
        pos[4] = pos[4] * 180 / np.pi
        pos[5] = pos[5] * 180 / np.pi
        return pos

    def DegToRad(self, pos):
        pos[0] = pos[0] * np.pi / 180
        pos[1] = pos[1] * np.pi / 180
        pos[2] = pos[2] * np.pi / 180
        pos[3] = pos[3] * np.pi / 180
        pos[4] = pos[4] * np.pi / 180
        pos[5] = pos[5] * np.pi / 180
        return pos

    def ikin_moveit(self, pos, eef_link_name):
        """

        :param pos: geometry_msgs/Pose with quaternion
        :param eef_link_name:
        :return: result: moveit_msgs/GetPositionIKResponse
        """
        try:
            rospy.wait_for_service('/dsr01m1013/compute_ik', timeout=5)
        except:
            return False
        try:
            robot = RobotState()
            robot.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            robot.joint_state.position = [0, 0, 0, 0, 0, 0]
            pose = PoseStamped()
            pose.pose = pos
            ik_rqst = PositionIKRequest(ik_link_name=eef_link_name, robot_state=robot, group_name='arm',
                                        pose_stamped=pose)
            ikin_srv = rospy.ServiceProxy('/dsr01m1013/compute_ik', GetPositionIK)
            result = ikin_srv(ik_rqst)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def ikin(self, pos, sol_space=0, ref=0):
        """

        :param pos: position in Metre and radian
        :param sol_space: default 0 (cf: doc doosan)
        :param ref: default world 0
        :return: position in deg
        """
        # convert pos angle from rad to deg and m to mm and deg
        pos = self.MRadToMMDeg(pos)
        try:
            rospy.wait_for_service('/dsr01m1013/motion/ikin', timeout=5)
        except:
            return False

        try:
            ikin_srv = rospy.ServiceProxy('/dsr01m1013/motion/ikin', Ikin)
            result = ikin_srv(pos, sol_space, ref)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def fkin_moveit(self, poses, eef_link_name):
        """

        :param poses:
        :param pos: position in radian
        :param ref: default world 0
        :return: pos in metre and radian

        """
        try:
            rospy.wait_for_service('/dsr01m1013/compute_fk', timeout=5)
        except:
            return False
        try:
            robot = RobotState()
            robot.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            robot.joint_state.position = poses
            msg = GetPositionFK()
            fkin_srv = rospy.ServiceProxy('/dsr01m1013/compute_fk', GetPositionFK)
            result = fkin_srv(fk_link_names=[eef_link_name], robot_state=robot)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def fkin(self, poses):
        """

        :param poses:
        :param pos: position in degree
        :param ref: default world 0
        :return: pos in metre and radian

        """
        try:
            rospy.wait_for_service('/dsr01m1013/compute_fk', timeout=5)
        except:
            return False
        try:
            robot = RobotState()
            robot.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            robot.joint_state.position = poses
            msg = GetPositionFK()
            fkin_srv = rospy.ServiceProxy('/dsr01m1013/compute_fk', GetPositionFK)
            result = fkin_srv(fk_link_names=['link6'], robot_state=robot)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def go_to_j(self, pos):
        """

        :param pos: position in radian
        :return: True if the command have been send, False if the service down
        """
        result = False

        # Spécifier les angles de joint souhaités pour chaque joint
        joint_angles = pos

        # Planifier une trajectoire de joint en utilisant les angles de joint
        result = self.move_group.go(joint_angles, wait=True)
        # if result:
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return result

    def go_to_l(self, pos):
        """
        This methode compute and execute the planification with a moveJ for the doosan
        :param pos: position cart in metre and radian (quaternion), this position is always from the ref base_0
                    and it place the tcp link at this pose.
        :return: True if a plan is found (and execute), False if no plan found
        """
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = pos.position.x
        pose_goal.position.y = pos.position.y
        pose_goal.position.z = pos.position.z

        pose_goal.orientation.x = pos.orientation.x
        pose_goal.orientation.y = pos.orientation.y
        pose_goal.orientation.z = pos.orientation.z
        pose_goal.orientation.w = pos.orientation.w

        self.move_group.set_pose_target(pose_goal)

        is_plan_valid = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # # It is always good to clear your targets after planning with poses.
        # # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)
        return is_plan_valid

    def dsr_go_relatif(self, pos):
        """

        :param pos:array 1*6
        :return:
        """
        # pos = self.MRadToMMDeg(pos)
        print(pos)
        try:
            rospy.wait_for_service('/dsr01m1013/motion/move_line', timeout=5)
        except:
            return False
        try:
            go_srv = rospy.ServiceProxy('/dsr01m1013/motion/move_line', MoveLine)
            result = go_srv(pos, [0, 0], [0, 0], 2, 0, 1, 1, 1, 1)
            print("msg go to send")
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")
            return False

    def dsr_spiral(self, revolution, max_radius, time):
        #         rosservice call /dsr01m1013/motion/move_spiral "revolution: 5.0
        # maxRadius: 10.0
        # maxLength: 0.0
        # vel: [0.0, 0.0]
        # acc: [0.0, 0.0]
        # time: 4.0
        # taskAxis: 2
        # ref: 0
        # syncType: 0"

        try:
            rospy.wait_for_service('/dsr01m1013/motion/move_spiral', timeout=5)
        except:
            return False
        try:
            spiral_srv = rospy.ServiceProxy('/dsr01m1013/motion/move_spiral', MoveSpiral)
            result = spiral_srv(revolution, max_radius, 0, [0, 0], [0, 0], time, 2, 0, 0)
            print("msg spiral to send")
            return result
        except rospy.ServiceException as e:
            return False
            print("Service call failed: s")

    def set_compliance(self):
        """
        :return:
        """
        try:
            rospy.wait_for_service('/dsr01m1013/force/task_compliance_ctrl', timeout=5)
        except:
            return False
        try:
            set_compliance_srv = rospy.ServiceProxy('/dsr01m1013/force/task_compliance_ctrl', TaskComplianceCtrl)

            result = set_compliance_srv([3000, 3000, 3000, 0, 0, 0], 1, 10)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def release_compliance(self):
        """
        :return:
        """
        try:
            rospy.wait_for_service('/dsr01m1013/force/release_compliance_ctrl', timeout=5)
        except:
            return False
        try:
            release_compliance_srv = rospy.ServiceProxy('/dsr01m1013/force/release_compliance_ctrl',
                                                        ReleaseComplianceCtrl)

            result = release_compliance_srv()
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def get_pos_x(self):
        """
        :return: a list for tcp position in rad and metre
        """
        try:
            rospy.wait_for_service('/dsr01m1013/system/get_current_pose', timeout=5)
        except:
            return False
        try:
            get_pos_srv = rospy.ServiceProxy('/dsr01m1013/system/get_current_pose', GetCurrentPose)
            # ask in space type 1 (task space)
            result = get_pos_srv(1)
            return self.MMDegToMRad(list(result.pos))
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def get_pos_j(self):
        """
        :return: a list for joints position in deg
        """
        try:
            rospy.wait_for_service('/dsr01m1013/aux_control/get_current_posj', timeout=5)
        except:
            return False
        try:
            get_posj_srv = rospy.ServiceProxy('/dsr01m1013/aux_control/get_current_posj', GetCurrentPosj)
            result = get_posj_srv()
            return list(result.pos)
        except rospy.ServiceException as e:
            print("Service get current pos j call failed: s")

    def get_current_rotm(self):
        """
        :return: the rotation matrix from the base to the tcp
        """
        try:
            rospy.wait_for_service('/dsr01m1013/aux_control/get_current_rotm', timeout=5)
        except:
            return False
        try:
            get_rtom_srv = rospy.ServiceProxy('/dsr01m1013/aux_control/get_current_rotm', GetCurrentRotm)
            # ask in space type 1 (base space)
            result = get_rtom_srv(0)
            rospy.loginfo(result)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def check_motion(self):
        try:
            rospy.wait_for_service('/dsr01m1013/motion/check_motion', timeout=5)
        except:
            return False
        try:
            check_motion_srv = rospy.ServiceProxy('/dsr01m1013/motion/check_motion', CheckMotion)
            result = check_motion_srv()
            return result.status
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def is_pt_reachable(self, pos):
        '''
        This methode check if the point is reachable for the robot.
        This methode check only the position, not the orientation
        :param pos: list the position and or the orientation in meter and radian
        :return: boolean
        '''
        ikin_r = self.ikin(pos, 0, 0)
        fkin_r = self.fkin(ikin_r.conv_posj, 0)

        ar = 2
        fkin_pos = [round(fkin_r[0], ar), round(fkin_r[1], ar), round(fkin_r[2], ar)]

        x = round(pos[0], ar)
        y = round(pos[1], ar)
        z = round(pos[2], ar)
        # u = round(x, ar)
        # v = round(y, ar)
        # w = round(z, ar)
        posD = [x, y, z]
        if (fkin_pos == posD):
            return True
        else:
            return False

    def is_at_pos_asked(self, pos):
        '''
        This methode compare the Doosan position from a position desired. We compare with a precision of 3 decimals
        :param pos: list of 6 position/orientation in metre and rad
        :return: the answer
        '''

        current_pos = self.get_pos_x()
        ar = 2
        current_pos = [round(current_pos[0], ar), round(current_pos[1], ar), round(current_pos[2], ar),
                       round(current_pos[3], ar), round(current_pos[4], ar), round(current_pos[5], ar)]
        pos = [round(pos[0], ar), round(pos[1], ar), round(pos[2], ar),
               round(pos[3], ar), round(pos[4], ar), round(pos[5], ar)]
        if current_pos == pos:
            return True
        else:
            return False

    def get_actual_manipulability(self):
        """
        Calculate the manipulability
        :return:
        """
        jac = np.asarray(self.move_group.get_jacobian_matrix(self.DegToRad(self.get_pos_j())))
        mani = np.sqrt(np.linalg.det(jac @ jac.T))
        print(mani)

    def check_collision(self, poses):
        """
        :param poses: joints poses in rad
        :return:
        """
        try:
            rospy.wait_for_service('/dsr01m1013/check_state_validity', timeout=1)
        except:
            return False
        try:
            robot = RobotState()
            robot.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            robot.joint_state.position = poses
            check_srv = rospy.ServiceProxy('/dsr01m1013/check_state_validity', GetStateValidity)
            result = check_srv(group_name='arm', robot_state=robot)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: s")

    def add_machine_colision(self, mesh_path, pose):
        """

        :param mesh_path:
        :return:
        """
        self.scene.add_mesh("machine", pose, mesh_path)

    def go_home(self):
        """
        Send the doosan to home pose
        :return: bool
        """
        res = self.go_to_j([0, -0.610865, 2.35619, 3.1415, 0, -3.05433])
        return res

    def is_plan_valid(self, joints):

        joint_state = JointState()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.position = joints
        plan = self.move_group.plan(joint_state)
        if plan[0]:
            return True
        else:
            return False

    def clear_collisions(self):
        """
        Clear the collision octomap
        :return:
        """
        try:
            rospy.wait_for_service('/dsr01m1013/clear_octomap', timeout=1)
        except:
            return False
        try:
            rospy.wait_for_service('/dsr01m1013/clear_octomap', timeout=1)
            clear_octomap = rospy.ServiceProxy('/dsr01m1013/clear_octomap', Empty)
            clear_octomap()
            return True
        except rospy.ServiceException as e:
            print("Service call failed: s")
            return False

    def get_joint_limit_index(self, configuration, k=1):
        in_exp = 1
        for index in range(5):
            in_exp = in_exp * (((configuration[index] - self.joint_limit[index][0]) * (self.joint_limit[index][1] - configuration[index])) /
                               (self.joint_limit[index][1] - self.joint_limit[index][0]))
        P = 1 - math.exp(- k * in_exp)
        return P

    def get_manipulability(self, configuration):
        """
        Calculate the manipulability
        :return:
        """
        jac = np.asarray(self.move_group.get_jacobian_matrix(configuration))
        mani = np.sqrt(np.linalg.det(jac @ jac.T))
        # print(mani)
        return mani

    def get_joint_limit(self):
        # tree =
        # Charger le fichier URDF
        root = ET.fromstring(rospy.get_param("/dsr01m1013/robot_description"))
        # root = tree.getroot()

        # Parcourir les éléments du fichier URDF
        joint_limits = []
        for joint in root.iter("joint"):
            # Vérifier si le joint a une limite
            if joint.find("limit") is not None:
                limit = joint.find("limit")
                lower_limit = float(limit.attrib["lower"])
                upper_limit = float(limit.attrib["upper"])
                joint_limits.append([lower_limit, upper_limit])
        return joint_limits
