#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)

    self.coords_pub2 = rospy.Publisher("coords_topic2", Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  # ADDED CODE
  def detect_yellow(self, image):
    mask = cv2.inRange(image, (0, 100, 100), (50, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # im1 = cv2.imshow('window2', mask)
    # cv2.waitKey(10000)

    M = cv2.moments(mask)
    if M['m00'] == 0: return np.array([-1, -1])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_blue(self, image):
    mask = cv2.inRange(image, (100, 0, 0), (255, 50, 50))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # im1 = cv2.imshow('window2', mask)
    # cv2.waitKey(10000)

    M = cv2.moments(mask)
    if M['m00'] == 0: return np.array([-1, -1])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_green(self, image):
    mask = cv2.inRange(image, (0, 100, 0), (50, 255, 50))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # im1 = cv2.imshow('window2', mask)
    # cv2.waitKey(10000)

    M = cv2.moments(mask)
    if M['m00'] == 0: return np.array([-1, -1])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_red(self, image):
    mask = cv2.inRange(image, (0, 0, 100), (50, 50, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # im1 = cv2.imshow('window2', mask)
    # cv2.waitKey(10000)

    M = cv2.moments(mask)
    if M['m00'] == 0: return np.array([-1, -1])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    # cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)
    # cv2.imshow('window2', image)
    # cv2.waitKey(10000)

    return np.array([cx, cy])

  def detect_orange(self, image):
    image_copy = image
    mask = cv2.inRange(image, (50, 100, 100), (100, 255, 255))

    edged = cv2.Canny(mask, 30, 200)

    wut1, cnts, wut2 = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    max_cunt = 0
    for cunts in cnts:
        if len(cunts) > max_cunt:
            max_cunt = len(cunts)
            cnt = cunts

    M = cv2.moments(cnt)
    if M['m00'] == 0: return np.array([-1, -1])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    cv2.circle(image_copy, (cx, cy), 2, (255, 255, 255), -1)
    # cv2.imshow('window2', image_copy)
    # cv2.waitKey(10000)

    return np.array([cx, cy])


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(50)

    # ADDED CODE
    y_coords = self.detect_yellow(self.cv_image2)
    b_coords = self.detect_blue(self.cv_image2)
    g_coords = self.detect_green(self.cv_image2)
    r_coords = self.detect_red(self.cv_image2)
    o_coords = self.detect_orange(self.cv_image2)

    self.joints2 = Float64MultiArray()
    self.joints2.data = np.append([], np.array([y_coords, b_coords, g_coords, r_coords, o_coords]))


    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))

      self.coords_pub2.publish(self.joints2)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


