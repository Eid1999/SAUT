#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
class odometry():
    def __init__(self):
        self.x=0
        self.y=0
        self.v_x=0
        self.v_y=0
        self.a_z=0
        self.theta=0
        self.time=0.5
        self.loop_rate = rospy.Rate(self.time**(-1))
        self.callback_active=0
        
    def update(self):
        
        rospy.Subscriber("/cmd_vel",Twist,self.callback)
        if self.callback_active:
            self.theta+=self.a_z*self.time
            self.x+=self.v_x*np.cos(self.theta)*self.time
            self.y+=self.v_x*np.sin(self.theta)*self.time
            rospy.loginfo(f"Position x:{self.x}, Position y:{self.y}, Orientation:{np.rad2deg(self.theta%2*np.pi)}")
            self.callback_active=0
    
        
        

       
    def callback(self, msg):
        self.v_x=msg.linear.x
        self.a_z=msg.angular.z
        self.callback_active=1
       


def main():
    rospy.init_node("Odometry", anonymous=True)
    rospy.loginfo("Odometry has started")
    state=odometry()    
    while not rospy.is_shutdown():  
        state.update()
        state.loop_rate.sleep()
        


if __name__ == "__main__":
    main()