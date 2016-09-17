import numpy as np
import cv2
import rospy
import rospkg
import sys
import struct
import ctypes
import math

import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

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
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        #self.camera_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)
        #self.camera_info_pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)
        #self.depth_pub = rospy.Publisher("/image_rect", Image, queue_size=10)
        self.pcl_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.pcl_callback)

    def camera_info_callback(self, camera_info):
        self.camera_info_pub.publish(camera_info)

    def pcl_callback(self, point_cloud):
        gen = pc2.read_points(point_cloud, skip_nans=False)
        h = point_cloud.header
        int_data = list(gen)
        print(len(int_data))
        #640 x 480 Point Cloud: 307200 if NaN is not skipped

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        i = 0
        j = 0
        for x in int_data:
            #print(i)
            if math.isnan(x[0]):
                x0 = 0
            else:
                x0 = x[0]

            if math.isnan(x[1]):
                x1 = 0
            else:
                x1 = x[1]
            if math.isnan(x[2]):
                x2 = 0
            else:
                x2 = x[2]
            frame[i][j] = [int(x0 * 256), int(x1 * 256), int(x2 * 256)]

            if j < 639:
                j = j + 1
            else:
                i = i + 1
                j = 0

        cv2.imshow("Point Cloud", frame)




    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.

        frame = np.array(frame, dtype=np.uint8)

        # Process the frame using the process_image() function
        #display_image = self.process_image(frame)
        lower = [0, 0, 0]
        upper = [90, 255, 255]
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)

        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(frame, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask=mask)
        # Display the image.

        output2 = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        #print(output)
        ret, thresh = cv2.threshold(output2, 120, 255, cv2.THRESH_BINARY)
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
            cv2.rectangle(frame, (self.x[i], self.y[i]), (self.x[i] + self.w[i], self.y[i] + self.h[i]), (255, 255, 255), 3)
            i = i + 1


        cv2.imshow(self.node_name, frame)

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
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
        except CvBridgeError, e:
            print e


        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)

        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        # Process the depth image
        depth_display_image = self.process_depth_image(depth_array)
        for i in range(0, len(self.x)-1):
            if self.x[i] is None:
                break
            cv2.rectangle(depth_display_image, (self.x[i]/2, self.y[i]/2), (self.x[i]/2 + self.w[i]/2, self.y[i]/2 + self.h[i]/2), (255, 255, 255), 3)
            cv2.rectangle(depth_display_image, (self.x[i]/2+320, self.y[i]/2), (self.x[i]/2+320 + self.w[i]/2, self.y[i]/2 + self.h[i]/2), (255, 255, 255), 3)
         #   print(repr(i) + "1:")
          #  print(depth_display_image[self.x[i]/2 + self.w[i] / 4][self.y[i]/2 + self.h[i] / 4])
         #   print(repr(i) + "2:")
         #   print(depth_display_image[160+ self.x[i] / 2 + self.w[i] / 4][160+ self.y[i] / 2 + self.h[i] / 4])
        # Display the result

        cv2.imshow("Depth Image", depth_display_image)

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





