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
        self.t=0
        self.theta=0
        self.t0=rospy.get_time()
        self.loop_rate = rospy.Rate(2)
        
    def update(self):
        rospy.Subscriber("/cmd_vel",Twist,self.callback)
        self.x=self.x+self.v_x*np.cos(self.theta)*self.t
        self.y=self.y+self.v_x*np.sin(self.theta)*self.t
        self.theta=self.theta+self.a_z
        rospy.loginfo(f"Position x:{self.x}, Position y:{self.y}")
        
        

       
    def callback(self, msg):
        self.v_x=msg.linear.x
        self.v_y=msg.linear.y
        self.a_z=msg.angular.z
        self.t=rospy.get_time()-self.t0
        self.t0=rospy.get_time()
       


def main():
    rospy.init_node("Odometry", anonymous=True)
    rospy.loginfo("Odometry has started")
    state=odometry()    
    while not rospy.is_shutdown():  
        state.update()
        state.loop_rate.sleep()
        


if __name__ == "__main__":
    main()