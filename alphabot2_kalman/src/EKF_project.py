#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
from fiducial_msgs.msg import FiducialTransformArray
from math import atan2
from scipy.spatial.transform import Rotation as R
import cv2

 
class Kalman_Filter():
    def __init__(self):
        
        self.Aruco_location={0:(1,2,0.5,np.pi/2),1:(5,1,0.5,np.pi/2),2:(2,1,0.5,np.pi/2),3:(4,3,0.5,np.pi/2),4:(2,3,np.pi/2),5:(10,2,0.5,np.pi/2),10:(1,1,0.5,np.pi/2)}
        rospy.init_node('kalman_filter', anonymous=True)
        # Time interval in seconds
        self.dk = 1
        
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
        self.process_noise_k_minus_1 = np.array([0.01,0.01,0.003,0.003,0.003])


        # State model noise covariance matrix
        self.R_k = np.array([[1,  0,   0, 0, 0],
                        [  0,1,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 ,1 ],
                        ])


        # Measurement matrix
        self.C_k = np.array([[1,  0,   0, 0, 0],
                        [  0,1,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 ,1 ],
                        ])

        # Sensor measurement noise covariance matrix
        # Measures how certain you are about the sensor data, R will be near zero if you trust it.
        #TRIAL AND ERROR! WE NEED TO RUN TESTS TO UNDERSTAND WHICH ARE THE BEST VALUES!
        self.Q_k =np.array([[5,  0,   0, 0, 0],
                        [  0,5,   0, 0, 0],
                        [  0,  0, 1,0 ,0 ],
                        [  0,  0, 0,1 ,0 ],
                        [  0,  0, 0,0 ,1 ],
                        ])

        # Sensor noise
        # NEEDS TESTING
        self.sensor_noise_k = np.array([0.07,0.07,0.04,0.04,0.04])

            
   
        # Observation by the sensors
        self.z_k_observation_vector=np.array([0,0,0,0,0])
            
        # x_k_minus_1
        self.state_estimate_k_minus_1 = np.array([0.0,0.0,0.0,0.0,0.0])
        
            
        #imput
        self.u_k_minus_1 = np.array([0.1,0.05])
            
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
            

    # Predict the state estimate at time k based on the state 
    # estimate at time k-1 and the control input applied at time k-1.
    def predict(self):

        B_estimate = np.array([[np.cos(self.state_estimate_k_minus_1[2])*self.dk, 0],
                           [np.sin(self.state_estimate_k_minus_1[2])*self.dk, 0],
                           [0, self.dk],
                           [1,0],
                           [0,1]])

        self.state_estimate_k = np.dot(self.A_k_minus_1, self.state_estimate_k_minus_1) + np.dot(B_estimate,self.u_k_minus_1) + self.process_noise_k_minus_1

                
        self.E_k = self.A_k_minus_1 @ self.E_k_minus_1 @ self.A_k_minus_1.T + (self.Q_k)
        
    

    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted 
    # the sensor measurements would be for the current timestep k.
    def update(self):

            measurement_residual_y_k = self.z_k_observation_vector - ((self.C_k @ self.state_estimate_k) + (self.sensor_noise_k))
            
            S_k = self.C_k @ self.E_k @ self.C_k.T + self.Q_k
        
            K_k = self.E_k @ self.C_k.T @ np.linalg.pinv(S_k)
                
            state_estimate_k = self.state_estimate_k + (K_k @ measurement_residual_y_k)
            self.estimated_x.append(state_estimate_k[0])
            self.estimated_y.append(state_estimate_k[1])
            self.estimated_ang.append(state_estimate_k[2])
            self.estimated_vel.append(state_estimate_k[3])
            self.estimated_ang_vel.append(state_estimate_k[4])
            
            self.E_k = self.E_k - (K_k @ self.C_k @ self.E_k)
    def callback(self,msg):
        xArray=[]
        yArray=[]
        orientationArray=[]
        if len(msg.transforms)!=0:
            for i in msg.transforms:
                msg=msg.transforms[-1]
                xAruco,yAruco,zAruco,angleAruco=self.Aruco_location[msg.fiducial_id]
                translation=msg.transform.translation
                rotation=msg.transform.rotation
                RmatrixAruco_world=R.from_euler('z', angleAruco, degrees=False).as_matrix()
                TmatrixAruco_world=np.array([xAruco,yAruco,zAruco])
                
                rotationMatrix_Camera_ARuco=R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
                position_Camera_Aruco=np.array([translation.x,translation.y,translation.z])
                position_Aruco_camera=-np.dot(position_Camera_Aruco,rotationMatrix_Camera_ARuco.transpose())
                postion_Camera_World=np.dot(position_Aruco_camera,RmatrixAruco_world)+TmatrixAruco_world
                rotation_Camera_Word=np.dot(rotationMatrix_Camera_ARuco.transpose(),RmatrixAruco_world)
                angle= R.from_matrix(rotation_Camera_Word).as_euler('zxy', degrees=False)[0]
               

               
                xArray.append(postion_Camera_World[0])
                yArray.append(postion_Camera_World[1])
                orientationArray.append(angle)
                
               
            x=sum(xArray)/len(xArray)
            y=sum(yArray)/len(yArray)
            orientation=sum(orientationArray)/len(orientationArray)
            
            obs_vel=np.sqrt((x-self.z_k_observation_vector[0])**2+(y-self.z_k_observation_vector[1])**2)/self.dk
            obs_ang_vel=(self.z_k_observation_vector[2]-orientation)/self.dk
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
    plot=0
    num_steps=100
    kalman=Kalman_Filter()
    
    
    while not rospy.is_shutdown():
    #for k in range(num_steps)
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,kalman.callback)
        if len(kalman.estimated_x)!=0:
            rospy.loginfo(f"\nPosition X:{kalman.estimated_x[-1]},\n Position Y:{kalman.estimated_y[-1]}\n Orientation:{kalman.estimated_ang[-1]} ")

        #kalman.callback(0)
        kalman.loop.sleep()

    timesteps = np.arange(num_steps)
    if plot!=0:
        plt.figure(figsize=(10, 6))
        plt.plot(timesteps, kalman.estimated_ang, label='Estimated')
        plt.plot(timesteps, kalman.observated_ang, label='Observed')
        plt.xlabel('Timesteps')
        plt.ylabel('ang/[rad]')
        plt.legend()
            
        plt.figure(figsize=(10, 6))
        plt.plot(timesteps, kalman.estimated_y, label='Estimated')
        plt.plot(timesteps, kalman.observated_y, label='Observed')
        plt.xlabel('Timesteps')
        plt.ylabel('Y/[m]')
        plt.legend()

        plt.figure(figsize=(10, 6))
        plt.plot(timesteps, kalman.estimated_x, label='Estimated')
        plt.plot(timesteps, kalman.observated_x, label='Observed')
        plt.xlabel('Timesteps')
        plt.ylabel('X/[m]')
        plt.legend()

        plt.show()


if __name__ == "__main__":
    main()
