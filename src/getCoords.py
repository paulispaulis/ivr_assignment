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


class Get_Coords:

  # Defines publisher and subscriber
  def __init__(self):
    
    self.coords1 = None
    self.coords2 = None

    rospy.init_node('coordCalc', anonymous=True)

    # initialize a publisher to send final coordinates for joints
    self.angle_pub = rospy.Publisher("final_angles", Float64MultiArray, queue_size=1)
    # initialize a subscriber to recieve coordinates from camera 1
    self.coords_sub1 = rospy.Subscriber("coords_topic1", Float64MultiArray, self.callback1)
    # initialize a subscriber to recieve coordinates from camera 2
    self.coords_sub2 = rospy.Subscriber("coords_topic2", Float64MultiArray, self.callback2)

    # Recieve the coordinates from camera 1
  def callback1(self,data):
    #array in the form of yellow, blue, green, red, orange
    self.coords1 = data.data
    self.merge_coords()
    
  #recieve coordinates from camera 2
  def callback2(self,data):
    self.coords2 = data.data
    self.merge_coords()
  
  def merge_coords(self):
      print(self.coords1)
      if self.coords1 is not None and self.coords2 is not None:
        #compute final coords
        pass
        

      # Publish the results
    #   try: 
    #     self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    #     self.coords_pub1.publish(coords1)
    #   except CvBridgeError as e:
    #     print(e)

# call the class
def main(args):
  #   fc = get_coords()
  #   try:
  #     rospy.spin()
  #   except KeyboardInterrupt:
  #     print("Shutting down")
  #   cv2.destroyAllWindows()
  # initialize the node named image_processing

  get_coords = Get_Coords()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

