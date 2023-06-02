#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from math import atan2
from scipy.spatial.transform import Rotation as R
import pylab as pl
from IPython.display import clear_output
import cv2
import collections
from sensor_msgs.msg import CameraInfo

from time import time


from filterpy.kalman import ExtendedKalmanFilter as EKF




    
class Kalman_Filter(EKF):
    def __init__(self):
        EKF.__init__(self, 5, 2, 0)
        self.Aruco_location={1:(0,85,20,np.pi),2:(481,85,20,0),9:(316,170,20,np.pi/2),3:(165,170,20,np.pi/2),8:(316,0,20,-np.pi/2),7:(165,0,20,-np.pi/2)}
        self.dt = 1
        self.x = np.array([[171,85, np.pi,0,0]]).T
        self.P = np.diag([.1, .1, .1,.1,.1])
        self.time=time()
               
        
        
        
        self.estimated_x=[]
        self.estimated_y=[]
        self.estimated_ang=[]
        self.predict_x=[]
        self.predict_y=[]
        
        rospy.init_node('kalman_filter', anonymous=True)
        # Time interval in seconds
        
        self.loop = rospy.Rate(1/self.dt)
        
        
        
        
        self.R_k = np.diag([1,  1 ])
        
    def state_model(self):
        x=self.x[0,0]
        y=self.x[1,0]
        theta=self.x[2,0]
        v=self.x[3,0]
        w=self.x[4,0]
        return np.array([[x+self.dt*v*np.cos(theta)],
                           [y+self.dt*v*np.sin(theta)],
                           [theta+w*self.dt],
                           [v],
                           [w]])
    def state_jacobian(self):
        x=self.x[0]
        y=self.x[1]
        theta=self.x[2,0]
        v=self.x[3,0]
        w=self.x[4]
        return np.array([[1,0,-v*self.dt*np.sin(theta),self.dt*np.cos(theta),0],
                         [0,1,v*self.dt*np.cos(theta),self.dt*np.sin(theta),0],
                         [0,0,1,0,self.dt],
                         [0,0,0,1,0],
                         [0,0,0,0,1]])
        
    def predict(self):
        self.x = self.state_model()
        
        F = self.state_jacobian()
        self.P = F @ self.P @ F.T
        self.predict_x.append(self.x[0])
        self.predict_y.append(self.x[1])

        
        
    def residual(self,a, b):
        
        y = a[:,None]- b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # move to [-pi, pi)
            y[1] -= 2 * np.pi
        return y

    def update(self,z,Aruco_pos):
            z_pred = self.measurement_model(self.x,Aruco_pos)
            y=self.residual(z,z_pred)
            Hx=self.measurement_jacobian(self.x,Aruco_pos)
            S = Hx @ self.P @ Hx.T + self.R_k
            K = self.P @ Hx.T @ np.linalg.inv(S)
            self.x = self.x+ K@ y
            I = np.eye(self.x.shape[0])
            self.P = (I - K@Hx)@ self.P

    
   
    def measurement_jacobian(self,x,landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]
        
        hyp = (px - x[0,0])**2 + (py - x[1,0])**2
        dist = np.sqrt(hyp)
        H=np.array([[-(px - x[0,0]) / dist,-(py - x[1,0]) / dist,0,0,0],[(py - x[1,0]) / hyp,-(px - x[0,0]) / hyp,-1,0,0]])
        return H
    
    def measurement_model(self,x, landmark_pos):

            px = landmark_pos[0]
            py = landmark_pos[1]
            dist = np.sqrt((px - x[0,0])**2 + (py - x[1,0])**2)

            Hx = np.array([[dist],
                    [atan2(py - x[1,0], px - x[0,0]) - x[2,0]]])
            return Hx
    

   
    def callback(self,msgs):
        if len(msgs.transforms)!=0 and time()-self.time>=self.dt:
            for  msg in msgs.transforms:
                xAruco,yAruco,zAruco,angleAruco=self.Aruco_location[msg.fiducial_id]
                translation=msg.transform.translation
                d = np.sqrt((translation.z)**2 + (translation.y)**2)  
                theta = np.arctan2(translation.y, translation.z)
                Aruco_camera=np.array([d,theta]).T
                Aruco_pos=np.array([xAruco,yAruco]).T
                self.predict()
                self.update(Aruco_camera,Aruco_pos)
                #self.update(Aruco_camera, HJacobian=self.measurement_jacobian, Hx=self.measurement_model, residual=self.residual,args=(Aruco_pos), hx_args=(Aruco_pos))
            self.estimated_x.append(self.x[0])
            self.estimated_y.append(self.x[1])
            self.estimated_ang.append(self.x[2])
            self.time=time()
        
    def listener(self):
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.callback)

        
        
        
        
    
        













def main():
    # Number of measurements
    #k=1
    
    kalman=Kalman_Filter()
   
    
    fig,ax=plt.subplots()
    line, = ax.plot([], [],label='Estimated')
    line2,=ax.plot([],[],label='Predicted')
    plt.ion()
    plt.show()
    while not rospy.is_shutdown():
        #rospy.Subscriber('/camera_info', CameraInfo, kalman.camera_info_callback)
        #rospy.Subscriber("/fiducial_vertices",FiducialArray,kalman.callback1)
        kalman.listener()
        
        
        if len(kalman.estimated_x)!=0:
            rospy.loginfo(f"\nPosition X:{kalman.estimated_x[-1]}\n Position Y:{kalman.estimated_y[-1]}\n Orientation:{np.rad2deg(kalman.estimated_ang[-1])%360} ")
            line.set_data(kalman.estimated_y, kalman.estimated_x)
            line2.set_data(kalman.predict_y, kalman.predict_x)
            ax.relim()
            ax.autoscale_view()
            plt.legend()
            plt.draw()
            plt.pause(0.0001)
            
            
        kalman.loop.sleep()
        

    


if __name__ == "__main__":
    main()
