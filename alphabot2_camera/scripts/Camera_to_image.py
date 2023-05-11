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
        self.loop_rate = rospy.Rate(1000)
        # Publishers
        #self.pub = rospy.Publisher('image', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image",Image,self.callback)


    def callback(self, msg):
        rospy.loginfo('Image received')
        self.image = self.br.imgmsg_to_cv2(msg)
    def aruco_detection(self):
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, self.aruco_ids, rejected) = cv2.aruco.detectMarkers(self.image, arucoDict,parameters=arucoParams)

def main():
    directory = "imgs"
    os.mkdir(directory)
    rospy.init_node("Image", anonymous=True)
    camera=image()
    rospy.loginfo("Image Node has started")
    i=0

    while not rospy.is_shutdown():            
        if camera.image is not None:
                rospy.loginfo('publishing image')
                #img.pub.publish(camera.br.cv2_to_imgmsg(camera.image))
                cv2.imwrite(os.path.join(directory, f"saved_img_{i}.jpg"), camera.image)
                camera.aruco_detection()
                i+=1
                camera.image= None
        camera.loop_rate.sleep()

if __name__ == "__main__":
    main()
    