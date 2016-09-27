#!/usr/bin/env python

import rospy, sys,tf
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose,Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import numpy as np

GROUP_NAME_ARM = 'Arm'
GROUP_NAME_GRIPPER = 'Hand'

GRIPPER_FRAME = 'tabletop_ontop'

GRIPPER_OPEN = [0] * 4
GRIPPER_CLOSED = [0,0.25,0.25,0.25]

GRIPPER_JOINT_NAMES = ['arm_5_joint', 'finger_joint_0', 'finger_joint_2', 'finger_joint_4']
GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'tabletop_ontop'


if __name__=='__main__': 
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_demo')
	scene = PlanningSceneInterface()
	robot = RobotCommander()
	jaco_arm = MoveGroupCommander("Arm")
	hand = MoveGroupCommander("Hand")
	rospy.sleep(1) 

	scene.remove_attached_object("6_hand_limb","box")
	# clean the scene 
	scene.remove_world_object("box") 

	# jaco_arm.set_named_target("resting") 
	# jaco_arm.go() 

	# hand.set_named_target("open") 
	# hand.go() 

	rospy.sleep(1) 

	# publish a demo scene 
	p = PoseStamped() 
	p.header.frame_id = "tabletop_ontop"

	# add an object to be grasped 
	p.pose.position.x = 0
	p.pose.position.y = 0
	p.pose.position.z = 0.05
	scene.add_box("box", p, (0.05, 0.05, 0.05)) 

	rospy.sleep(1) 

	#jaco_arm.pick("box")
	# grasps = [] 

	# g = Grasp() 
	# g.id = "test" 
	grasp_pose = PoseStamped() 
	grasp_pose.header.frame_id = REFERENCE_FRAME 
	grasp_pose.pose.position.x = 0
	grasp_pose.pose.position.y = 0.01 
	grasp_pose.pose.position.z = 0.25
	orient = Quaternion(*tf.transformations.quaternion_from_euler(-0.81473782016728, -1.5353834214261521, 0.5270780713957257))
	grasp_pose.pose.orientation = orient

	jaco_arm.set_pose_target(grasp_pose) 
	jaco_arm.go() 
	rospy.sleep(1)

	hand.set_joint_value_target([0,0.2,0.2,0.2])
	hand.go()

	rospy.sleep(1)

	scene.attach_box('6_hand_limb', 'box',touch_links = ['9_finger_index','9_pinkie_index','9_thumb_index'])

	rospy.sleep(1)


	grasp_pose.pose.position.z = 0.5
	jaco_arm.set_pose_target(grasp_pose) 
	jaco_arm.go()
	rospy.sleep(1)

	# rospy.sleep(1)

	# # set the grasp pose
	# g.grasp_pose = grasp_pose

	# # define the pre-grasp approach
	# g.pre_grasp_approach.direction.header.frame_id = REFERENCE_FRAME
	# g.pre_grasp_approach.direction.vector.x = 0.0
	# g.pre_grasp_approach.direction.vector.y = 0.0
	# g.pre_grasp_approach.direction.vector.z = -1.0
	# g.pre_grasp_approach.min_distance = 0.001
	# g.pre_grasp_approach.desired_distance = 0.1

	# g.pre_grasp_posture.header.frame_id = "6_hand_limb"
	# g.pre_grasp_posture.joint_names = GRIPPER_JOINT_NAMES

	# pos = JointTrajectoryPoint()
	# pos.positions = GRIPPER_OPEN

	# g.pre_grasp_posture.points.append(pos)

	# # set the grasp posture
	# g.grasp_posture.header.frame_id = "6_hand_limb"
	# g.grasp_posture.joint_names = GRIPPER_JOINT_NAMES

	# pos = JointTrajectoryPoint()
	# pos.positions = GRIPPER_CLOSED
	# pos.effort.append(0.0)

	# g.grasp_posture.points.append(pos)

	# # set the post-grasp retreat
	# g.post_grasp_retreat.direction.header.frame_id = REFERENCE_FRAME
	# g.post_grasp_retreat.direction.vector.x = 0.0
	# g.post_grasp_retreat.direction.vector.y = 0.0
	# g.post_grasp_retreat.direction.vector.z = 1.0
	# g.post_grasp_retreat.desired_distance = 0.25
	# g.post_grasp_retreat.min_distance = 0.01

	# g.allowed_touch_objects = ["box"]

	# g.max_contact_force = 0

	# # append the grasp to the list of grasps
	# grasps.append(g)

	# rospy.sleep(2)

	# # pick the object
	# jaco_arm.pick("box", grasps)

	rospy.spin()
	roscpp_shutdown()
