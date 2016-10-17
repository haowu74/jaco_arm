import numpy as np
import cv2
import rospy
import rospkg
import sys
import struct
import ctypes
import math
from sympy import *
import tf
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from PIL import Image as pilImage

import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from copy import deepcopy
import Tkinter as tki
from PIL import ImageTk

class Jaco_rapper():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.jaco_arm = MoveGroupCommander("Arm")
        self.hand = MoveGroupCommander("Hand")

    def pick(self, x, y, z, color, reference="arm_stand"):
        # clean the scene
        self.scene.remove_attached_object("6_hand_limb", "box")
        self.scene.remove_world_object("box")
        rospy.sleep(1)
        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = "tabletop_ontop"
        # add an object to be grasped
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = reference
        grasp_pose.pose.position.x = x
        grasp_pose.pose.position.y = y + 0.35
        grasp_pose.pose.position.z = z
        orient = Quaternion(
            *tf.transformations.quaternion_from_euler(-0.81473782016728, -1.5353834214261521, 0.5270780713957257))
        grasp_pose.pose.orientation = orient
        self.jaco_arm.set_pose_target(grasp_pose)
        self.jaco_arm.go()
        rospy.sleep(1)
        self.hand.set_joint_value_target([0, 0.2, 0.2, 0.2])
        self.hand.go()
        # rospy.sleep(1)
        # scene.attach_box('6_hand_limb', 'box',touch_links = ['9_finger_index','9_pinkie_index','9_thumb_index'])
        # rospy.sleep(1)
        grasp_pose.pose.position.y = 0.5
        self.jaco_arm.set_pose_target(grasp_pose)
        self.jaco_arm.go()

'''
To do:
Choose objects to pick, ignore the ones out of the boundary of the table (using ARM_STAND frame)


'''


