#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from fiducial_msgs.msg import FiducialTransformArray
from math import atan2
from scipy.spatial.transform import Rotation as R
import pylab as pl
from IPython.display import clear_output
import cv2
import collections

matplotlib.use('TkAgg')
class Kalman_Filter():
    def __init__(self):

        self.r0=0
        self.Aruco_location={1:(0,85,20,np.pi),2:(481,85,20,0),9:(316,170,20,np.pi/2),3:(165,170,20,np.pi/2),8:(316,0,20,-np.pi/2),7:(165,0,20,-np.pi/2)}
        rospy.init_node('kalman_filter', anonymous=True)
        # Time interval in seconds
        self.dk = 10
        
            # Node cycle rate (in Hz).
            
        self.loop = rospy.Rate(1/self.dk)
        
            # A matrix
        # Expresses how the state of the system [x,y,ang] changes when no control command is executed.
        self.A_k_minus_1 = np.array([[1,  0,   0, 0, 0],
                        [  0,1,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 ,1 ],
                        ])

        #Noise of the state space model
        self.process_noise_k_minus_1 = np.array([1,1,1,1,1]).transpose()*0.00001


        # State model noise covariance matrix
        self.R_k = np.array([[1,  0,   0, 0, 0],
                        [  0,1,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 ,1 ],
                        ])*2


        # Measurement matrix
        self.C_k = np.array([[1,  0,   0, 0, 0],
                        [  0,1,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 ,1 ],
                        ])*0.5

        # Sensor measurement noise covariance matrix
        # Measures how certain you are about the sensor data, R will be near zero if you trust it.
        #TRIAL AND ERROR! WE NEED TO RUN TESTS TO UNDERSTAND WHICH ARE THE BEST VALUES!
        self.Q_k =np.array([[1,  0,   0, 0, 0],
                        [  0,1,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 , 1],
                        ])

    

            
   
            
        # x_k_minus_1
        self.state_estimate_k_minus_1 = np.array([0.85,1.55,np.pi,0.0,0.0]).transpose()
        self.z_k_observation_vector=self.state_estimate_k_minus_1
        
            
    
            
        # State covariance matrix
        self.E_k_minus_1 = np.array([[0.1,  0,   0, 0, 0],
                        [  0,0.1,   0, 0, 0],
                        [  0,  0, 0.1,0 ,0 ],
                        [  0,  0, 0,0.1 ,0 ],
                        [  0,  0, 0,0 ,0.1 ],
                        ])
            

        self.estimated_x = []
        self.estimated_y = []
        self.estimated_ang = []
        self.observated_x=[]
        self.observated_y=[]
        self.observated_ang=[]
        self.estimated_vel = []
        self.estimated_ang_vel = []
        self.observated_vel=[]
        self.observated_ang_vel=[]
        self.predict_array=[]
            

    # Predict the state estimate at time k based on the state 
    # estimate at time k-1 and the control input applied at time k-1.
    def predict(self):

        B_estimate = np.array([self.state_estimate_k_minus_1[3]*np.cos(self.state_estimate_k_minus_1[2])*self.dk,
                           self.state_estimate_k_minus_1[3]*np.sin(self.state_estimate_k_minus_1[2])*self.dk,
                           self.dk*self.state_estimate_k_minus_1[4],
                           0,
                           0]).transpose()

        self.state_estimate_k = np.dot(self.A_k_minus_1, self.state_estimate_k_minus_1) + B_estimate+self.process_noise_k_minus_1
        self.predict_array.append(self.state_estimate_k)
                
        self.E_k = self.A_k_minus_1 @ self.E_k_minus_1 @ self.A_k_minus_1.T + self.R_k
        
    

    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted 
    # the sensor measurements would be for the current timestep k.
    def update(self):

            measurement_residual_y_k = self.z_k_observation_vector - (self.C_k @ self.state_estimate_k)
            
            S_k = self.C_k @ self.E_k @ self.C_k.T + self.R_k
        
            K_k = self.E_k @ self.C_k.T @ np.linalg.inv(S_k)
                
            state_estimate_k = self.state_estimate_k + (K_k @ measurement_residual_y_k)
            self.estimated_x.append(state_estimate_k[0])
            self.estimated_y.append(state_estimate_k[1])
            self.estimated_ang.append(state_estimate_k[2])
            self.estimated_vel.append(state_estimate_k[3])
            self.estimated_ang_vel.append(state_estimate_k[4])
            
            self.E_k = self.E_k - (K_k @ self.C_k @ self.E_k)
    def callback(self,msgs):
        xArray=[]
        yArray=[]
        orientationArray=[]
        if len(msgs.transforms)!=0:
            for  msg in msgs.transforms:
                xAruco,yAruco,zAruco,angleAruco=self.Aruco_location[msg.fiducial_id]
                translation=msg.transform.translation
                #rotation=msg.transform.rotation
                RmatrixAruco_world=R.from_euler('z', angleAruco, degrees=False).as_matrix()
                TmatrixAruco_world=(np.array([xAruco,yAruco,zAruco])/100).transpose()
                
                #rotationMatrix_Camera_ARuco=R.from_quat([rotation.x,rotation.y,rotation.z,rotation.w]).as_matrix()
                position_Camera_Aruco=np.array([translation.x,translation.y,-translation.z]).transpose()
                rotationMatrix_Camera_ARuco=R.from_euler('z', np.arctan2(translation.y,translation.x), degrees=False).as_matrix()
                
                
                position_Aruco_camera=-np.dot(rotationMatrix_Camera_ARuco.transpose(),position_Camera_Aruco)
                position_Camera_World=np.dot(RmatrixAruco_world,position_Aruco_camera)+TmatrixAruco_world
                rotation_Camera_Word=np.dot(rotationMatrix_Camera_ARuco.transpose(),RmatrixAruco_world)
                
                #position_Aruco_camera=position_Camera_Aruco
                #position_Camera_World=np.dot(RmatrixAruco_world.transpose(),position_Aruco_camera-TmatrixAruco_world)
                #rotation_Camera_Word=np.dot(rotationMatrix_Camera_ARuco,RmatrixAruco_world)
                
                
                
                angle= R.from_matrix(rotation_Camera_Word).as_euler('zxy', degrees=False)[0]
                xArray.append(position_Camera_World[0])
                yArray.append(position_Camera_World[1])
                orientationArray.append(angle)
                
            
            x=sum(xArray)/len(xArray)
            y=sum(yArray)/len(yArray)
            orientation=sum(orientationArray)/len(orientationArray)
            #rospy.loginfo(f"id:{msg.fiducial_id},pos:{x},{y},{orientation},{position_Camera_Aruco}")
            self.r1=np.sqrt((x-self.z_k_observation_vector[0])**2+(y-self.z_k_observation_vector[1])**2)
            obs_vel=(self.r1-self.r0)/self.dk
            self.r0=self.r1
            obs_ang_vel=(orientation-self.z_k_observation_vector[2])/self.dk
            self.predict()
            self.z_k_observation_vector= [x,y,orientation,obs_vel,obs_ang_vel]
            self.observated_x.append(self.z_k_observation_vector[0])
            self.observated_y.append(self.z_k_observation_vector[1])
            self.observated_ang.append(self.z_k_observation_vector[2])
            self.observated_vel.append(self.z_k_observation_vector[3])
            self.observated_ang_vel.append(self.z_k_observation_vector[4])
        
            self.update()
                
            self.state_estimate_k_minus_1 = self.state_estimate_k
            self.E_k_minus_1 = self.E_k
            
            
        

        
        
        



def main():
    # Number of measurements
    #k=1
    
    kalman=Kalman_Filter()
   
    
    fig,ax=plt.subplots()
    line, = ax.plot([], [],label='Estimated')
    #line2,=ax.plot([],[],label='Predicted')
    line1,=ax.plot([],[],label='Observed')
    plt.ion()
    plt.show()
    #plt.ion()
    
    while not rospy.is_shutdown():
    #for k in range(num_steps)
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,kalman.callback)
        if len(kalman.estimated_x)!=0:
            rospy.loginfo(f"\nPosition X:{kalman.estimated_x[-1]}\n Position Y:{kalman.estimated_y[-1]}\n Orientation:{kalman.estimated_ang[-1]} ")
            line.set_data(kalman.estimated_y, kalman.estimated_x)
            line1.set_data(kalman.observated_y, kalman.observated_x)
            #line2.set_data(kalman.predict_array[1], kalman.predict_array[0])
            ax.relim()
            ax.autoscale_view()
            plt.legend()
            plt.draw()
            plt.pause(0.0001)
            

        #kalman.callback(0)
            
        kalman.loop.sleep()
        

    


if __name__ == "__main__":
    main()
