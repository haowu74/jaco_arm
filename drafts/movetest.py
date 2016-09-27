#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, random
import moveit_msgs.msg
import geometry_msgs.msg
import time

class Wrapper:
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("All")
        #self.group.set_planner_id("RRTkConfigDefault")
        self.group.set_planning_time(15)
        #self.group.set_start_state_to_current_state()
        #self.group.set_end_effector_link("jaco_6_hand_limb")
        self.robot = moveit_commander.RobotCommander()
        #self.pos_sub = rospy.Subscriber("/my_pos", )

    def setPose(self, deltaX, deltaY, deltaZ):
        #self.group.set_pose_reference_frame("arm_stand")
        print self.robot.get_group_names()
        current = self.group.get_current_pose("jaco_6_hand_limb").pose
        print(self.group.get_planning_frame())
        print(self.group.get_pose_reference_frame())

        print(current)
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.stamp = rospy.Time.now()
        pose_target.header.frame_id = "/world"

        pose_target.pose.orientation.w = 1#current.orientation.w
        pose_target.pose.orientation.x = 0#current.orientation.x
        pose_target.pose.orientation.y = 0#current.orientation.y
        pose_target.pose.orientation.z = 0#current.orientation.z
        #pose_target.orientation.w = 0.17
        #pose_target.orientation.x = -0.01
        #pose_target.orientation.y = 0.98
        #pose_target.orientation.z = 0
        pose_target.pose.position.x = 0.45 #0.248448447363 #current.position.x + deltaX
        pose_target.pose.position.y = 0.05 #0.0541192093905 #current.position.y + deltaY
        pose_target.pose.position.z = 1.12 #0.723365499473 #current.position.z + deltaZ
        #pose_target.pose.position.x = 0.89
        #pose_target.pose.position.y = -0.35
        #pose_target.pose.position.z = 1.2
        print("set pose-----------------")
        print(self.group.get_current_pose("jaco_6_hand_limb").pose)
        self.group.set_pose_target(pose_target, "jaco_6_hand_limb")
        plan1 = self.group.plan()
        self.group.go(True)
        #self.group.clear_pose_targets()
        print(pose_target)

        '''
        group_variable_values = self.group.get_current_joint_values()
        group_variable_values[0] = 1.0
        self.group.set_joint_value_target(group_variable_values)
        plan2 = self.group.plan()
        self.group.go(True)
        '''

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('jaco_cli', anonymous=True)
    argv = rospy.myargv(argv=sys.argv)  # filter out any arguments used by ROS
    if len(argv) != 4:
        print "usage: r2_cli.py  Xdelta Ydelta Zdelta"
        sys.exit(1)
    r2w = Wrapper()
    r2w.setPose(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    print("after moving pose----------")
    # print(r2w.group.group.get_current_pose().pose)
    # print "============ Reference frame: %s" % r2w.group.get_planning_frame()
    # print "end effector: %s" % r2w.group.get_end_effector_link()



    while True:
        time.sleep(2)

    moveit_commander.roscpp_shutdown()