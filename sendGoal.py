#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, random
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown

FRAME = 'tabletop_ontop'

class Wrapper:
    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander("Arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group.set_pose_reference_frame(FRAME)
        self.robot = RobotCommander()
        rospy.sleep(1)


    def setPose(self,X,Y,Z):
        orient = Quaternion(*tf.transformations.quaternion_from_euler(-0.81473782016728, -1.5353834214261521, 0.5270780713957257))

        pose_target = PoseStamped()
        pose_target.header.frame_id = FRAME
        pose_target.pose.orientation = orient

        pose_target.pose.position.x = X

        pose_target.pose.position.y = Y

        pose_target.pose.position.z = Z

        self.group.set_pose_target(pose_target)
        self.group.go(True)
        rospy.sleep(2)

    def getState(self):
        print(self.robot.get_current_state())

    def getPose(self) :
        print(self.group.get_current_pose().pose)
    
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv) 
    rospy.init_node('jaco_cli',anonymous=True)
    argv = rospy.myargv(argv=sys.argv) # filter out any arguments used by ROS 
    if len(argv) != 4:
        print "usage: r2_cli.py  Xdelta Ydelta Zdelta"
        sys.exit(1)
    r2w = Wrapper()
    r2w.setPose(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))

    moveit_commander.roscpp_shutdown()
