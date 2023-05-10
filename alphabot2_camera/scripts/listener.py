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
    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')

            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
                cv2.imwrite("img.jpg", self.image)
            self.loop_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("imagetimer111", anonymous=True)
    img=image()
    img.start()