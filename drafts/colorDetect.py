import numpy as np
import cv2
import rospy
import rospkg
import sys
import struct
import ctypes
import math
import tf
import io

import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import PoseStamped

class colorDetect:
    x= [None] * 100
    y= [None] * 100
    w = [None] * 100
    h = [None] * 100
    def __init__(self):
        self.node_name = "colorDetect"

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        # And one for the depth image
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Depth Image", 25, 350)

        # And one for the depth image
        cv.NamedWindow("Point Cloud", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Point Cloud", 400, 350)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.image_callback)
        #self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        #self.camera_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)
        #self.camera_info_pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)
        #self.depth_pub = rospy.Publisher("/image_rect", Image, queue_size=10)
        #self.pcl_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.pcl_callback)
        #self.pos_pub = rospy.Publisher('/my_pos', PoseStamped, queue_size=100)
        self.transListen = tf.TransformListener()

    def camera_info_callback(self, camera_info):
        self.camera_info_pub.publish(camera_info)

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
        # Publish the new pose.



    def pcl_callback(self, point_cloud):
        gen = pc2.read_points(point_cloud, skip_nans=False)
        h = point_cloud.header
        int_data = list(gen)
        print(len(int_data))


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

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        '''
                index = 0
                for row in frame:
                    for x in row:
                        print index, ':', x
                        index = index + 1
        '''
        #frame = np.array(frame, dtype=np.uint8)
        #exit(0)
        # Process the frame using the process_image() function
        #display_image = self.process_image(frame)
        lower = np.array([90, 100, 100])
        upper = np.array([125, 255, 255])
        #lower = np.array(lower, dtype=np.uint8)
        #upper = np.array(upper, dtype=np.uint8)

        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(frame, lower, upper)

        moments = cv2.moments(mask)
        vert_centre = moments['m01'] / moments['m00'];
        horz_centre = moments['m10'] / moments['m00'];

        output = cv2.bitwise_and(frame, frame, mask=mask)
        # Display the image.

        output1 = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        output2 = cv2.cvtColor(output1, cv2.COLOR_BGR2GRAY)
        #print(output)
        ret, thresh = cv2.threshold(output2, 10, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        i = 0
        for contour in contours:
            cnt = contour
        #M = cv2.moments(cnt)
        #area = cv2.contourArea(cnt)

            x0, y0, w0, h0 = cv2.boundingRect(cnt)
            if w0 * h0 < 10:
                continue
            self.x[i] = (x0)
            self.y[i] = (y0)
            self.w[i] = (w0)
            self.h[i] = (h0)
            cv2.rectangle(output2, (self.x[i], self.y[i]), (self.x[i] + self.w[i], self.y[i] + self.h[i]), (255, 255, 255), 3)
            i = i + 1

        cv2.rectangle(frame_bgr, (0, 0), (10, 5), (255, 255, 255), 3)
        cv2.imshow(self.node_name, frame_bgr)

        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def depth_callback(self, ros_image):

        #self.depth_pub.publish(ros_image)
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv2(ros_image)
        except CvBridgeError, e:
            print e


        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array_normal = np.zeros((480, 640), dtype=np.float32)
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array_normal, 0, 1, cv2.NORM_MINMAX)

        # Process the depth image
        depth_display_image = self.process_depth_image(depth_array_normal)
        for i in range(0, len(self.x)-1):
            if self.x[i] is None:
                break
            #cv2.rectangle(depth_display_image, (self.x[i]/2, self.y[i]/2), (self.x[i]/2 + self.w[i]/2, self.y[i]/2 + self.h[i]/2), (255, 255, 255), 3)
            #cv2.rectangle(depth_display_image, (self.x[i]/2+320, self.y[i]/2), (self.x[i]/2+320 + self.w[i]/2, self.y[i]/2 + self.h[i]/2), (255, 255, 255), 3)
            #cv2.rectangle(depth_display_image, (self.x[i], self.y[i]), (self.x[i] + self.w[i], self.y[i] + self.h[i]),
            #                  (255, 255, 255), 3)

                #   print(repr(i) + "1:")
          #  print(depth_display_image[self.x[i]/2 + self.w[i] / 4][self.y[i]/2 + self.h[i] / 4])
         #   print(repr(i) + "2:")
         #   print(depth_display_image[160+ self.x[i] / 2 + self.w[i] / 4][160+ self.y[i] / 2 + self.h[i] / 4])
        # Display the result

        cv2.imshow("Depth Image", depth_display_image)
        #cubic1 = (self.x[0] + self.h[0] / 2) /2 * 320 + (self.y[0] + self.w[0] / 2) / 2
        #cubic2 = (self.x[1] + self.h[1] / 2) /2 * 320 + (self.y[1] + self.w[1] / 2) / 2
        print("d0")
        #print(depth_array[(self.x[0] + self.h[0] / 2) /2][(self.y[0] + self.w[0] / 2) / 2])
        print(depth_array[(self.x[0]+self.w[0]/2)][(self.y[0]+self.h[0]/2)])
        #print(depth_array[160][120])
        print("d1")

        #print(depth_array[(self.x[1] + self.h[1] / 2) /2][(self.y[1] + self.w[1] / 2) / 2])
        print(depth_array[(self.x[1]+self.w[1]/2)][(self.y[1]+self.h[1]/2)])


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

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        colorDetect()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)





