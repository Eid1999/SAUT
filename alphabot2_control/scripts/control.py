#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


class control:
    def __init__(self):
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)
        # Publishers
        self.pub = rospy.Publisher('alphabot2/cmd_vel', Twist,queue_size=10)

    def message_creation(l_x=0,l_y=0,l_z=0,a_x=0,a_y=0,a_z=0):
        self.msg=Twist()
        self.msg.linear.x=l_x
        self.msg.linear.y=l_y
        self.msg.angular.y=a_y
        self.msg.angular.x=a_x
def main():
    rospy.init_node("Motor Control", anonymous=True)
    motor=control()
    rospy.loginfo("Control Node has started")
    
    while not rospy.is_shutdown():  
        message_creation(l_x=2.0,a_z=1.0)        
        motor.pub.publish(msg)
        motor.loop_rate.sleep()


if __name__ == "__main__":
    main()