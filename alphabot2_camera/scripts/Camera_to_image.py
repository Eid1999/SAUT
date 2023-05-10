#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os


class image:
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color",Image,self.callback)


    def callback(self, msg):
        rospy.loginfo('Image received')
        self.image = self.br.imgmsg_to_cv2(msg)
        

def main():
    rospy.init_node("Image", anonymous=True)
    img=image()
    rospy.loginfo("Image Node has started")
    
    while not rospy.is_shutdown():            
        if img.image is not None:
                rospy.loginfo('publishing image')
                img.pub.publish(img.br.cv2_to_imgmsg(img.image))
        img.loop_rate.sleep()

if __name__ == "__main__":
    main()
    