#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform
import random

import numpy as np
import matplotlib.pyplot as plt
	
import time


def message_creation(i):
    message=FiducialTransform()
    message.fiducial_id=i
    message.transform.translation.x=9*(random.random())+1 
    message.transform.translation.y=9*(random.random())+1 
    messageArray=FiducialTransformArray()
    messageArray.transforms.append(message)
    
    return messageArray
def main():
    start=time.time()
    aruco_id=[0,1,2,3,4,5]
    rospy.init_node('aruco_simulator', anonymous=True)
    loop_rate = rospy.Rate(0.5)
    pub_fiducial = rospy.Publisher('/fiducial_transforms', FiducialTransformArray,queue_size=10)
    i=random.choice(aruco_id)
    while not rospy.is_shutdown():
        i=random.choice(aruco_id)
        message=message_creation(i)
        pub_fiducial.publish(message)
        loop_rate.sleep()
           

if __name__ == "__main__":
    main()
