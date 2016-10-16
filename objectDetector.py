#!/usr/bin/env python
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
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import Tkinter as tki
from PIL import Image as PilImage
from PIL import ImageTk



MAX_OBJECTS = 10

GROUP_NAME_ARM = 'Arm'

GROUP_NAME_GRIPPER = 'Hand'

GRIPPER_FRAME = 'tabletop_ontop'

GRIPPER_OPEN = [0] * 4

GRIPPER_CLOSED = [0, 0.25, 0.25, 0.25]
GRIPPER_JOINT_NAMES = ['arm_5_joint', 'finger_joint_0', 'finger_joint_2', 'finger_joint_4']
GRIPPER_EFFORT = [1.0]
STATE_CALIBRATE = 0
STATE_OPERATE = 1


class Object:
    COLOR_NONE = "None"
    SHAPE_NONE = "None"
    rect = False

    def __init__(self):
        self.x = -1
        self.y = -1
        self.w = 0
        self.h = 0
        self.color = self.COLOR_NONE  # Colour should have a name
        self.shape = self.SHAPE_NONE  # Shape should have a name
        self.COLOR_RANGES = {"pink": {"low": np.array([145, 100, 100]), "high": np.array([165, 255, 255])},
                             "yellow": {"low": np.array([20, 100, 100]), "high": np.array([35, 255, 255])},
                             "green": {"low": np.array([45, 100, 50]), "high": np.array([75, 255, 255])},
                             "blue": {"low": np.array([90, 100, 100]), "high": np.array([110, 255, 255])},
                             "white": {"low": np.array([0, 0, 100]), "high": np.array([20, 0, 255])},
                             "red": {"low": np.array([160, 100, 100]), "high": np.array([179, 255, 255])}
                             }


class Jaco_rapper():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.jaco_arm = MoveGroupCommander("Arm")
        self.hand = MoveGroupCommander("Hand")

    def pick(self, x, y, z, reference="arm_stand"):
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


class calibration:
    points = []
    pointsRef = []
    # cameraPos
    # cameraOrient
    done = False

    def calPos(self):
        x = Symbol('x')
        y = Symbol('y')
        z = Symbol('z')
        x01 = self.pointsRef[0][0]
        y01 = self.pointsRef[0][1]
        z01 = self.pointsRef[0][2]
        x02 = self.pointsRef[1][0]
        y02 = self.pointsRef[1][1]
        z02 = self.pointsRef[1][2]
        x03 = self.pointsRef[2][0]
        y03 = self.pointsRef[2][1]
        z03 = self.pointsRef[2][2]

        x11 = self.points[0][0]
        y11 = self.points[0][1]
        z11 = self.points[0][2]
        x12 = self.points[1][0]
        y12 = self.points[1][1]
        z12 = self.points[1][2]
        x13 = self.points[2][0]
        y13 = self.points[2][1]
        z13 = self.points[2][2]

        #Simulation
        x11 = 0.0348
        y11 = 0.0681
        z11 = 1.072

        x01 = 0
        y01 = -0.035
        z01 = -0.035

        x12 = 0.0748
        y12 = 0.1571
        z12 = 0.61

        x02 = 0.075
        y02 = -0.035
        z02 = -0.5

        x13 = 0.178
        y13 = 0.0755
        z13 = 1.053

        x03 = 0.15
        y03 = -0.035
        z03 = -0.035

        rsq1 = x11 ** 2 + y11 ** 2 + z11 ** 2
        rsq2 = x12 ** 2 + y12 ** 2 + z12 ** 2
        rsq3 = x13 ** 2 + y13 ** 2 + z13 ** 2

        try:
            results = solve([(x - x01) ** 2 + (y - y01) ** 2 + (z - z01) ** 2 - rsq1,
                            (x - x02) ** 2 + (y - y02) ** 2 + (z - z02) ** 2 - rsq2,
                            (x - x03) ** 2 + (y - y03) ** 2 + (z - z03) ** 2 - rsq3], [x, y, z])

        except:
            print "calculating position failed"
            return

        for result in results:
            if result[1] > 0:
                self.cameraPos = (result[0], result[1], result[2])
                print self.cameraPos
                break

        x21 = x01 - result[0]
        y21 = y01 - result[1]
        z21 = z01 - result[2]
        #yaw = math.atan(y21 / x21) - math.atan(y11 / x11)
        #pitch = math.atan(x21 / z21) - math.atan(x11 / z11)
        #roll = math.atan(z21 / y21) - math.atan(z11 / y11)

        v1 = np.array([x21, y21, z21], dtype=np.float64)
        v2 = np.array([x11, y11, z11], dtype=np.float64)

        #rotate angle
        a1 = np.linalg.norm(v1)
        a2 = np.linalg.norm(v2)
        b1 = np.dot(v1/a1, v2/a2)
        c1 = np.clip(b1, -1.0, 1.0)
        angle = np.arccos(c1)
        #angle = np.arccos(np.clip(np.dot(v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)), -1.0, 1.0))

        #rotate axis
        v = np.cross(v1, v2)
        vlen = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        cosx = v[0] / vlen
        cosy = v[1] / vlen
        cosz = v[2] / vlen


        w = math.cos(angle/2)
        x = math.sin(angle/2)*cosx
        y = math.sin(angle/2)*cosy
        z = math.sin(angle/2)*cosz
        self.cameraOrient = (w, x, y, z)

        print self.cameraOrient

        '''
        e0 = Symbol('e0')
        e1 = Symbol('e1')
        e2 = Symbol('e2')
        e3 = Symbol('e3')

        z11 = math.sqrt(x21 * x21 + y21 * y21 + z21 * z21 - x11 * x11 - y11 * y11)

        try:
            results = solve([x21*(e0**2+e1**2-0.5)+y21*(e1*e2+e0*e3)+z21*(e1*e3-e0*e2)-x11*0.5,
                             x21*(e1*e2-e0*e3)+y21*(e0**2+e2**2-0.5)+z21*(e2*e3+e0*e1)-y11*0.5,
                             x21*(e1*e3+e0*e2)+y21*(e2*e3-e0*e1)+z21*(e0**2+e3**2-0.5)-z11*0.5,
                             e0**2+e1**2+e2**2+e3**2-1], [e0, e1, e2, e3])
        except:
            print "calculating angle failed"
            return
        for result in results:
            self.cameraOrient = (result[0],result[1], result[2], result[3])
        '''
        self.done = True


class objectDetect:
    objects = []
    containers = []

    disableImageSub = False
    disablePclSub = False

    def __init__(self, feature="Color"):
        self.cubicPose2 = PoseStamped()
        #self.jaco_rapper = Jaco_rapper()
        self.calibration = calibration()
        self.node_name = "objectDetector"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.image_callback)
        self.cameraInfo_sub = rospy.Subscriber("/kinect2/sd/camera_info", CameraInfo, self.cameraInfo_callback)
        self.pcl_sub = rospy.Subscriber("/kinect2/sd/points", PointCloud2, self.pcl_callback)

        self.pos_pub = rospy.Publisher('/my_pos', PoseStamped, queue_size=100)
        self.info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=100)

        self.transListen = tf.TransformListener()

        self.root = tki.Tk()
        self.panel = None

        #self.cv_window_name = self.node_name
        #cv.NamedWindow("rgb", cv.CV_WINDOW_NORMAL)
        #cv.MoveWindow("rgb", 25, 75)

        self.COLOR_RANGES = {"pink": {"low": np.array([145, 100, 100]), "high": np.array([165, 255, 255])},
                             "yellow": {"low": np.array([20, 100, 100]), "high": np.array([35, 255, 255])},
                             "green": {"low": np.array([45, 100, 50]), "high": np.array([75, 255, 255])},
                             "blue": {"low": np.array([90, 100, 100]), "high": np.array([110, 255, 255])},
                             "white": {"low": np.array([0, 0, 200]), "high": np.array([180, 255, 255])},
                             "red": {"low": np.array([160, 100, 100]), "high": np.array([179, 255, 255])}
                             }

        self.state = STATE_CALIBRATE

        self.tk_setup()

    def tk_setup(self):
        self.tk_create_widgets()
        self.tk_setup_layout()


    def tk_create_widgets(self):
        #self.panel = tki.Label(self.root)
        self.button_yes = tki.Button(self.root, text="Yes", command = self.show_calibrate_xyz)
        self.button_no = tki.Button(self.root, text="No", command = self.calibrate_next)
        self.textbox_x = tki.Entry(self.root)
        self.textbox_y = tki.Entry(self.root)
        self.textbox_z = tki.Entry(self.root)
        self.lbl = tki.Label(self.root)




    def tk_setup_layout(self):
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_columnconfigure(0, weight=0)
        self.root.grid_columnconfigure(4, weight=0)
        #self.panel.grid(row=0, columnspan=5, rowspan=5)
        self.lbl.grid(row = 5, columnspan=5)
        self.button_yes.grid(row = 6, column = 1)
        self.button_no.grid(row = 6, column = 3)


    def show_calibrate_xyz(self):
        self.textbox_x.grid(row = 7, column = 0)
        self.textbox_y.grid(row = 7, column = 2)
        self.textbox_z.grid(row = 7, column = 4)

    def calibrate_next(self):
        pass


    def image_callback(self, image):
        try:
            # image = self.uncompressImage(compressedImage, "bgr8")
            image_bgr = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        rects = []
        colors = []
        #boxes = []
        #Find the objects with colors
        for k in self.COLOR_RANGES:
            if k != "white":
                rs = self.findColor(image, k, 20)
                rects.extend(rs)
                n = len(rs)
                for i in range(0, n):
                    colors.append(k)

        #boxes = self.findColor(image, "white", 20)


        if not self.disableImageSub:
            self.objects[:] = []
            #num = 0
            i = 0
            for rect in rects:
                if rect[2] * rect[3] > 100:
                    object = Object()
                    object.x = rect[0]
                    object.y = rect[1]
                    object.w = rect[2]
                    object.h = rect[3]
                    object.color = colors[i]
                    #object.id = num
                    self.objects.append(object)
                    i += 1
                    #num += 1
                    #cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (255, 255, 255), 3)

        #Mark the detected object with white rectangles
        for object in self.objects:
            if object.rect:
                cv2.rectangle(image_bgr, (object.x, object.y), (object.x + object.w, object.y + object.h), (255, 255, 255), 3)
                cv2.putText(image_bgr, object.color, (object.x, object.y - 20), cv.CV_FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255))
        '''
        for box in boxes:
            if box[2] * box[3] > 500:
                container = Object()
                container.x = box[0]
                container.y = box[1]
                container.w = box[2]
                container.h = box[3]
                # object.id = num
                self.containers.append(container)
                # num += 1
                cv2.rectangle(image_bgr, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 0, 0), 3)

        '''

        cv2.imwrite("rgb.png", image_bgr)
        # cv2.drawContours(image, contours, -1, (255, 255, 255), 10)

        cv2.imshow("rgb", image_bgr)
        cv.WaitKey(5)
        #rgb_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        #rgb_image = PilImage.fromarray(rgb_image)
        #rgb_image = ImageTk.PhotoImage(rgb_image)


        '''
        if self.panel is None:
            self.panel = tki.Label(image=rgb_image)
            self.panel.image = rgb_image
            self.panel.pack(side="left", padx=10, pady=10)

        else:
            self.panel.configure(image=rgb_image)
            self.panel.image = rgb_image
        '''

    def cameraInfo_callback(self, cameraInfo):

        self.info_pub.publish(cameraInfo)

    def pcl_callback(self, point_cloud):

        gen = pc2.read_points(point_cloud, skip_nans=False)
        width = point_cloud.width
        height = point_cloud.height

        int_data = list(gen)
        pclimage = np.zeros((height, width, 3), dtype=np.float32)


        i = 0
        for x in int_data:
            # print r, g, b
            if math.isnan(x[0]) or math.isnan(x[1]) or math.isnan(x[2]):
                x = (0, 0, 0)
            pclimage[i / width][i % width] = (x[0], x[1], x[2])
            i = i + 1

        for object in self.objects:
            # print "({}, {}): ({})".format(object.y + object.h/2, object.x + object.w/2, image[object.y + object.h/2][object.x + object.w/2])
            position = pclimage[object.y + object.h / 2][object.x + object.w / 2]

            if position[2] < 1.2 and position[2] <> 0:
                print "Start"
                print "({}, {}): ({})".format(object.y + object.h / 2, object.x + object.w / 2,
                                              pclimage[object.y + object.h / 2][object.x + object.w / 2])
                print "End"

                if self.state == STATE_CALIBRATE:
                    self.disableImageSub = True
                    object.rect = True
                    self.find_own_pos(position[0], position[1], position[2])
                    object.rect = False
                    self.disableImageSub = False

                elif (self.state == STATE_OPERATE) :
                    self.disableImageSub = True
                    object.rect = True
                    self.transform(position[0], position[1], position[2])
                    object.rect = False
                    self.disableImageSub = False

                else:
                    print "({}), ({})".format(self.calibration.cameraPos, self.calibration.cameraOrient)

    '''
    def uncompressImage(self, data, encoding='pass'):
        # From https://github.com/ros-perception/vision_opencv/blob/indigo/cv_bridge/python/cv_bridge/core.py
        buf = np.ndarray(shape=(1, len(data.data)), dtype=np.uint8, buffer=data.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
        if encoding == 'pass':
            return im
        from cv_bridge.boost.cv_bridge_boost import cvtColor2
        return cvtColor2(im, "bgr8", encoding)
    '''

    def transform(self, x, y, z):
        cubicPose = PoseStamped()
        now = rospy.Time.now()

        cubicPose.header.stamp = now
        cubicPose.header.frame_id = '/calibration'
        cubicPose.pose.position.x = x
        cubicPose.pose.position.y = y
        cubicPose.pose.position.z = z
        cubicPose.pose.orientation.x = 0.0
        cubicPose.pose.orientation.y = 0.0
        cubicPose.pose.orientation.z = 0.0
        cubicPose.pose.orientation.w = 1.0

        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0.285, -1.12),
                         (0, -0.069721168, 0.997566518, 0),
                         now,
                         "calibration",
                         "arm_stand")

        self.transListen.waitForTransform('calibration', 'arm_stand', now, rospy.Duration(4.0))
        self.cubicPose2 = self.transListen.transformPose('arm_stand', cubicPose)
        self.pos_pub.publish(self.cubicPose2)
        print(self.cubicPose2)
        pick_bool = raw_input()
        if(int(pick_bool) == 1) :
            self.jaco_rapper.pick(self.cubicPose2.pose.position.x, self.cubicPose2.pose.position.y, self.cubicPose2.pose.position.z)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

    def findColor(self, image, color, minSize):
        rect = [0] * 4
        rects = []
        low = self.COLOR_RANGES[color]["low"]
        high = self.COLOR_RANGES[color]["high"]
        mask = cv2.inRange(image, low, high)
        output = cv2.bitwise_and(image, image, mask=mask)
        output1 = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        output2 = cv2.cvtColor(output1, cv2.COLOR_BGR2GRAY)
        # print(output)
        ret, thresh = cv2.threshold(output2, 10, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        i = 0
        for contour in contours:
            cnt = contour
            x0, y0, w0, h0 = cv2.boundingRect(cnt)
            if w0 * h0 < minSize:
                continue
            rect = [x0, y0, w0, h0]
            rects.append(rect)
            i = i + 1
        num = i
        return rects

    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        return edges

    def process_depth_image(self, frame):
        # Just return the raw image for this demo

        return frame

    # Calibrate the camera's position
    def find_own_pos(self, x, y, z):

        #print "Detected calibration position:"
        #lbl = tki.Label(self.root, text = "Detected calibration position \n ({}, {}, {}). Use it?".format(x, y, z))
        self.lbl.configure(text = "Detected calibration position \n ({}, {}, {}). Use it?".format(x, y, z))
        #lbl.pack(side="bottom", fill="both", expand="yes", padx=10, pady=10)
        #print "({}, {}, {})".format(x, y, z)
        '''
        cal = ''
        while cal != 'y' and cal != 'Y' and cal != 'n' and cal != 'N':
            cal = raw_input("Use this point? (y/n)")
            if cal == 'y' or cal == 'Y':
                break
            elif cal == 'n' or cal == 'N':
                return

        btn_yes = tki.Button(self.root, text = "Yes")
        btn_no = tki.Button(self.root, text = "No")
        btn_yes.pack(side="bottom", fill="both", expand="yes", padx=0, pady=0)
        btn_no.pack(side="bottom", fill="both", expand="yes", padx=0, pady=0)
        '''
        while True:
            pass

        print "Enter the coordination in the frame used by the Arm:"
        x0 = y0 = z0 = ''
        while not is_number(x0):
            #x0 = Float(raw_input("x: "))
            x0 = 0
        while not is_number(y0):
            y0 = 0
            #= Float(raw_input("y: "))
        while not is_number(z0):
            z0 = 0
                #= Float(raw_input("z: "))

        self.calibration.points.append((x, y, z))
        self.calibration.pointsRef.append((x0, y0, z0))

        if len(self.calibration.points) == 3:
            self.calibration.calPos()

        if self.calibration.done:
            self.state = STATE_OPERATE

        return


def main(args):
    try:
        obj = objectDetect()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

    #obj.root = tki.Tk()
    #obj.panel = None
    obj.root.mainloop()
    #since we have mainloop(), rospy.spin() is not required

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

if __name__ == '__main__':
    main(sys.argv)