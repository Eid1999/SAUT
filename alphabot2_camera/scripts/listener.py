#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class image:
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color",Image,self.callback)


    def callback(self, msg):
        rospy.loginfo('Image received')
        self.image = self.br.imgmsg_to_cv2(msg)
def main():
     while not rospy.is_shutdown():

        if self.IMGS.image is not None:
            self.IMGS.pub.publish(self.br.cv2_to_imgmsg(self.image))
        self.IMGS.loop_rate.sleep()
        robot.kalman()


if __name__ == "__main__":
    main()