#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from waveshare_alphabot2.msg import Pan_Tilt
#fiducial_slam

class control:
    def __init__(self):
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(2)
        # Publishers
        self.pub_motor = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.pub_camera = rospy.Publisher('/pan_tilt', Pan_Tilt,queue_size=10)

    def message_creation(self,l_x=0,l_y=0,l_z=0,a_x=0,a_y=0,a_z=0):
        self.msg=Twist()
        self.msg.linear.x=l_x
        self.msg.linear.y=l_y
        self.msg.angular.z=a_z
        
        self.camera_msg = Pan_Tilt()
        self.camera_msg.pan = 0.5
        self.camera_msg.tilt = 0.5
        
def main():
    rospy.init_node("Motor_Control", anonymous=True)
    motor=control()
    rospy.loginfo("Control Node has started")
    
    while not rospy.is_shutdown():  
        motor.message_creation(l_x=0.2,l_y=.5,a_z=1.0)
        rospy.loginfo("Publishing Linear and angular velocities")        
        motor.pub_motor.publish(motor.msg)
        motor.pub_camera.publish(motor.camera_msg)
        motor.loop_rate.sleep()


if __name__ == "__main__":
    main()