import numpy as np
import cv2
import rospy
import rospkg
import sys
import struct
import ctypes
import math

import tf
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped

MAX_OBJECTS = 10

class object:
    COLOR_NONE = "None"
    SHAPE_NONE = "None"

    def __init__(self):
        self.x = -1
        self.y = -1
        self.w = 0
        self.h = 0
        self.color = self.COLOR_NONE     #Colour should have a name
        self.shape = self.SHAPE_NONE     #Shape should have a name
        self.COLOR_RANGES = {"pink": {"low": np.array([145, 100, 100]), "high": np.array([165, 255, 255])},
                             "yellow": {"low": np.array([20, 150, 150]), "high": np.array([35, 255, 255])},
                             "green": {"low": np.array([50, 100, 100]), "high": np.array([90, 255, 255])},
                             "blue": {"low": np.array([90, 150, 150]), "high": np.array([110, 255, 255])},
                             "white": {"low": np.array([0, 0, 100]), "high": np.array([20, 0, 255])}
                             }


class objectDetect:
    objects = []
    def __init__(self, feature = "Color"):
        self.node_name = "objectDetector"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.depth_callback)
        self.cameraInfo_sub = rospy.Subscriber("/kinect2/hd/camera_info", CameraInfo, self.cameraInfo_callback)
        self.pcl_sub = rospy.Subscriber("/points", PointCloud2, self.pcl_callback)
        self.pos_pub = rospy.Publisher('/my_pos', PoseStamped, queue_size=100)
        self.image_pub = rospy.Publisher('/image_rect', Image, queue_size=100)
        self.info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=100)


        self.transListen = tf.TransformListener()

        self.cv_window_name = self.node_name
        cv.NamedWindow("rgb", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("rgb", 25, 75)

        # And one for the depth image
        cv.NamedWindow("depth", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("depth", 25, 350)

        cv.NamedWindow("pcl", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("pcl", 100, 350)

        #Test color
        white = np.uint8([[[255, 255, 255]]])
        hsv_white = cv2.cvtColor(white, cv2.COLOR_BGR2HSV)
        self.COLOR_RANGES = {"pink": {"low": np.array([145, 100, 100]), "high": np.array([165, 255, 255])},
                             "yellow": {"low": np.array([20, 150, 150]), "high": np.array([35, 255, 255])},
                             "green": {"low": np.array([50, 100, 100]), "high": np.array([90, 255, 255])},
                             "blue": {"low": np.array([90, 150, 150]), "high": np.array([110, 255, 255])},
                             "white": {"low": np.array([0, 0, 100]), "high": np.array([20, 0, 255])}
                             }

        #return

    def image_callback(self, image):
        try:
            #image = self.uncompressImage(compressedImage, "bgr8")
            image_bgr = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        rects = self.findColor(image, "green", 20)
        self.objects[:] = []
        for rect in rects:
            object.x = rect[0]
            object.y = rect[1]
            object.w = rect[2]
            object.h = rect[3]
            self.objects.append(object)
            cv2.rectangle(image, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]),
                          (255, 255, 255), 3)

        #cv2.drawContours(image, contours, -1, (255, 255, 255), 10)
        cv2.imshow("rgb", image)
        self.keystroke = cv.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def depth_callback(self, image):
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError, e:
            print e
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array_normal = np.zeros((1080, 1920), dtype=np.float32)
        cv2.normalize(depth_array, depth_array_normal, 0, 1, cv2.NORM_MINMAX)
        depth_display_image = self.process_depth_image(depth_array_normal)

        for object in self.objects:
            print "({}, {}): {}".format(object.y + object.h/2, object.x + object.w/2, depth_array[object.y + object.h/2][object.x + object.w/2])
            #print depth_array[object.y + object.h/2][object.x + object.w/2]

        cv2.imshow("depth", depth_display_image)
        self.image_pub.publish(image)

    def cameraInfo_callback(self, cameraInfo):
        self.info_pub.publish(cameraInfo)

    def pcl_callback(self, point_cloud):
        gen = pc2.read_points(point_cloud, skip_nans=False)
        int_data = list(gen)
        image = np.zeros((1080, 1920, 3), dtype=np.float32)
        i = 0
        for x in int_data:
            #print r, g, b
            if math.isnan(x[0]) or math.isnan(x[1]) or math.isnan(x[2]):
                x= (0, 0, 0)
            image[i/1920][i%1920] = (x[0], x[1], x[2])
            i = i+1

        for object in self.objects:
            print "({}, {}): ({})".format(object.y + object.h/2, object.x + object.w/2, image[object.y + object.h/2][object.x + object.w/2])

        '''
        h = point_cloud.header
        int_data = list(gen)
        print(len(int_data))

        image = np.zeros((1080, 1920), dtype=np.uint8)
        for i in range (0, 1080):
            for j in range(0, 1920):
                image[i][j] = [ord(point_cloud.data[(1920 * j + i) * 3]), ord(point_cloud.data[(1920 * j + i) * 3 + 1]), ord(point_cloud.data[(1920 * j + i) * 3 + 2])]
        '''
        #cv2.imshow("pcl", image)
        '''
        cubic1 = [self.x[0] + self.w[0] / 2, self.y[0] +  self.h[0] / 2]
        cubic2 = [self.x[1] + self.w[1] / 2, self.y[1] +  self.h[1] / 2]
        print("pcl")
        p1 = int_data[int(cubic1[1] * 640 + cubic1[0])]
        p2 = int_data[int(cubic2[1] * 640 + cubic2[0])]
        print(p1)
        print(p2)
        #print(int_data[153600])
        self.transform(0, 0, 0)
        self.transform(p1[0],p1[1], p1[2])
        self.transform(p2[0],p2[1], p2[2])
        '''

    def uncompressImage(self, data, encoding='pass'):
        # From https://github.com/ros-perception/vision_opencv/blob/indigo/cv_bridge/python/cv_bridge/core.py
        buf = np.ndarray(shape=(1,len(data.data)), dtype=np.uint8, buffer=data.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
        if encoding == 'pass':
            return im
        from cv_bridge.boost.cv_bridge_boost import cvtColor2
        return cvtColor2(im, "bgr8", encoding)

    def transform(self, z, y, x):
        cubicPose = PoseStamped()
        cubicPose.header.stamp = rospy.Time.now()
        cubicPose.header.frame_id = '/openni_camera_link'
        cubicPose.pose.position.x = x
        cubicPose.pose.position.y = y
        cubicPose.pose.position.z = z
        cubicPose.pose.orientation.x = 0.0
        cubicPose.pose.orientation.y = 0.0
        cubicPose.pose.orientation.z = 0.0
        cubicPose.pose.orientation.w = 1.0
        now = rospy.Time.now()

        self.transListen.waitForTransform('/openni_camera_link', 'tabletop_ontop', now, rospy.Duration(4.0))
        cubicPose2 = self.transListen.transformPose('tabletop_ontop', cubicPose)
        self.pos_pub.publish(cubicPose2)

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
        i= 0
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

def main(args):
    try:
        objectDetect()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
