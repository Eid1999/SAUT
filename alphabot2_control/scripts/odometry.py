#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
class odometry():
    def __init__(self):
        self.x=0
        self.y=0
        self.t0=rospy.get_time()
        self.loop_rate = rospy.Rate(10)
        
    def update(self):
        rospy.Subscriber("alphabot2/cmd_vel",Twist,self.callback)
        self.x=self.x+self.v_x*np.cos(self.a_z)*self.t
        self.y=self.y+self.v_y*np.cos(self.a_z)*self.t
        rospy.loginfo(f"Position x:{self.x}, Position y:{self.y}")
        
        

       
    def callback(self, msg):
        rospy.loginfo('Image received')
        self.v_x=msg.linear.x
        self.v_y=msg.linear.y
        self.a_z=msg.angular.z
        self.t=rospy.get_time()-self.t0
        self.t0=rospy.get_time()
       


def main():
    rospy.init_node("Odometry Calculation", anonymous=True)
    rospy.loginfo("Odometry has started")
    state=odometry()    
    while not rospy.is_shutdown():  
        state.update()
        state.loop_rate.sleep()
        


if __name__ == "__main__":
    main()