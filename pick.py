#!/usr/bin/env python

import rospy, sys,tf
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose,Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import numpy as np

GROUP_NAME_ARM = 'Arm'
GROUP_NAME_GRIPPER = 'Hand'

GRIPPER_FRAME = 'tabletop_ontop'

GRIPPER_OPEN = [0.0] * 6
GRIPPER_CLOSED = [0.15,0.15,0.15,0.6981,0.6981,0.6981]

GRIPPER_JOINT_NAMES = ['finger_joint_0','finger_joint_1','finger_joint_2','finger_joint_3','finger_joint_4','finger_joint_5']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'tabletop_ontop'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the jaco arm
        jaco_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Get the name of the end-effector link
        end_effector_link = jaco_arm.get_end_effector_link()
 
        # Allow some leeway in position (meters) and orientation (radians)
        jaco_arm.set_goal_position_tolerance(0.05)
        jaco_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        jaco_arm.allow_replanning(True)
        
        # Set the jaco arm reference frame_id
        jaco_arm.set_pose_reference_frame("tabletop_ontop")
        
        # Allow 5 seconds per planning attempt
        jaco_arm.set_planning_time(5)
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5
                
        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name        
        box1_id = 'box1'
        tool_id = 'tool'
                
        # Remove leftover objects from a previous run
        scene.remove_world_object(box1_id)

        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, box1_id)
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)
        
        # Set the height of the table off the ground
        table_ground = 0.00
        
        # Set the dimensions of the scene objects [l, w, h]
        box1_size = [0.1, 0.05, 0.05]
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = "tabletop_ontop"
        box1_pose.pose.position.x = 0
        box1_pose.pose.position.y = 0
        box1_pose.pose.position.z = box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        # Make the table red and the boxes orange
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        # Send the colors to the planning scene
        self.sendColors()
        # Initialize the grasp pose to the box1 pose
        grasp_pose = box1_pose
       # Generate a list of grasps
        grasp = self.make_grasps(grasp_pose, [box1_id])

        # Publish the grasp poses so they can be viewed in RViz
        self.gripper_pose_pub.publish(grasp.grasp_pose)
        rospy.sleep(0.2)
    
        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = jaco_arm.pick(box1_id, grasp)
            rospy.sleep(0.2)
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        g.id = "MY Grasp"
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.4, 0.01, [0.0, 1.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.6, 0.01, [0.0, 0.0, 1.0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
        
        orient = Quaternion(*tf.transformations.quaternion_from_euler(0, -1.4, 0))
        g.grasp_pose.pose.orientation = orient

        # Set the allowed touch objects to the input list
        g.allowed_touch_objects = allowed_touch_objects
        # Degrade grasp quality for increasing pitch angles
        g.grasp_quality = 1.0
                    
        # Return the list
        return g
    
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItDemo()

    
