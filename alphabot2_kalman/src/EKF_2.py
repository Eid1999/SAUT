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





class Kalman_Filter():

    def __init__(self):
        # EKF.__init__(self, 5, 2, 0)
        self.Aruco_location = {0: (10, 0), 1: (125, 0), 2: (300, 0), 3: (398, 0), 4: (514, 0), 5: (606, 0), 6: (693, 0), 7: (809, 0), 8: (888, 0), 9: (995, 0),
                               10: (1111, 0), 11: (1191, 0), 12: (1269, 0), 13: (1385, 0), 14: (1466, 0), 15: (1547, 0), 16: (1628, 0), 17: (1638, 10),
                               18: (1638, 156), 19: (1638, 249), 20: (1638, 365), 21: (1638, 445), 22: (1638, 525), 23: (1638, 641), 24: (1638, 751),
                               25: (1638, 861), 26: (1638, 971), 27: (1638, 1081), 28: (1638, 1271), 29: (1638, 1417), 30: (1638, 1508), 31: (1638, 1652),
                               32: (1628, 1662), 33: (1538, 1662), 34: (1448, 1662),35: (1358.0, 1662),36: (1358.0, 1662),37: (1268.0, 1662),38: (1122.0, 1662),
                               39: (1029.0, 1662),40:(936,1662),41:(820,1662),42:(729,1662),43:(638,1662),44:(545,1662),45:(398,1662),46:(318,1662),47:(238,1662),
                               48:(158,1662),49:(0,1653),50:(0,1506),51:(0,1416),52:(0,1326),53:(0,1224),54:(0,1077),55:(0,976),56:(0,862),57:(0,766),58:(0,670),
                               59:(0,484),60:(0,364),61:(0,244),62:(0,62),63:(179,169),64:(279,169),65:(379,169),66:(479,169),67:(580,169),68:(680,169),69:(788,169),
                               70:(888,169),71:(988,169),72:(1088,169),73:(1188,169),74:(1288,169),75:(1383,169),76:(1393,179),77:(1393,224),78:(1393,335.3),79:(1393,431.3),
                               80:(1393,533.3),81:(1393,629.3),82:(1393,711.3),83:(1393,779.3),84:(1393,907.3),85:(1393,977.3),86:(1393,1038.3),87:(1393,1134.3),
                               88:(1393,1236.3),89:(1393,1332.3),90:(1393,1411.3),91:(1393,1444.4),92:(1400,1454.3),93:(1312,1454.3),94:(1224,1454.3),95:(1116,1454.3),96:(1022,1454.3),
                               97:(928,1454.3),98:(811,1454.3),99:(672,1454.3),100:(621,1454.3),101:(539,1454.3),102:(443,1454.3),103:(341,1454.3),104:(245,1454.3),105:(179,1454.3),
                               106:(169,1444.3),107: (169, 1358.3),108: (169, 1272.3),109: (169, 1186.3),110: (169, 1100.3),111: (169, 1014.3),112: (169, 928.3),113: (169, 892.3),
                               114: (169, 816.3),115: (169, 740.3),116: (169, 664.3),117: (198, 653.8),118: (217, 670.8),119: (227.5, 681.3),120: (287.5, 681.3),121: (457.5, 681.3),
                               122: (648.5, 681.3),123: (762.5, 681.3),124: (773, 670.8),125: (773, 587.8),126: (773, 504.8),127: (773, 421.8),128: (746, 393.8),129: (670, 393.8),
                               130: (574, 393.8),131: (512, 393.8),132: (436, 393.8),133: (332, 383.3),134: (332, 410.3),135: (313, 420.8),136: (295, 370.8),137: (295, 296.8),
                               138: (295, 222.8)}
        # # 6 Arucos
        #self.Aruco_location={1:(0,85,),2:(481,85),9:(316,170),3:(165,170),8:(316,0),7:(165,0)}
        self.t = .5
        self.x = np.array([[0, 0, 0, 0, 0]]).T
        # Initial Uncertanty of the state variables .50^2 m, (pi/4)^2 radians,10^2 m/s, (pi/40)^2 radians/s
        self.P = np.diag([1.0**2, 1.0**2, (np.pi/4)
                         ** 2, .80**2, (np.pi/40)**2])
        self.time = time()

        self.estimated_x = []
        self.estimated_y = []
        self.estimated_ang = []
        self.predict_x = []
        self.predict_y = []

        rospy.init_node('kalman_filter', anonymous=True)
        # Time interval in seconds

        self.loop = rospy.Rate(1/self.t)

        #self.R_k = np.diag([.2,  0.008])
        self.R_k = np.diag([.06,   0.006])

    def state_model(self):
        x = self.x[0, 0]
        y = self.x[1, 0]
        theta = self.x[2, 0]
        v = self.x[3, 0]
        w = self.x[4, 0]
        return np.array([[x+self.dt*v*np.cos(theta)],
                         [y+self.dt*v*np.sin(theta)],
                         [theta+w*self.dt],
                         [v],
                         [w]])

    def state_jacobian(self):
        x = self.x[0]
        y = self.x[1]
        theta = self.x[2, 0]
        v = self.x[3, 0]
        w = self.x[4]
        return np.array([[1, 0, -v*self.dt*np.sin(theta), self.dt*np.cos(theta), 0],
                         [0, 1, v*self.dt*np.cos(theta),
                          self.dt*np.sin(theta), 0],
                         [0, 0, 1, 0, self.dt],
                         [0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 1]])

    def predict(self):
        self.x = self.state_model()

        F = self.state_jacobian()
        self.P = F @ self.P @ F.T
        self.predict_x.append(self.x[0])
        self.predict_y.append(self.x[1])
        

    def residual(self, a, b):

        y = a[:, None] - b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # move to [-pi, pi)
            y[1] -= 2 * np.pi
        return y

    def update(self, z, Aruco_pos):
        z_pred = self.measurement_model(self.x, Aruco_pos)
        y = self.residual(z, z_pred)
        Hx = self.measurement_jacobian(self.x, Aruco_pos)
        S = Hx @ self.P @ Hx.T + self.R_k
        K = self.P @ Hx.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(self.x.shape[0])
        self.P = (I - K@Hx) @ self.P

    def measurement_jacobian(self, x, landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]

        hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
        dist = np.sqrt(hyp)
        H = np.array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0, 0, 0],
                     [(py - x[1, 0]) / hyp, -(px - x[0, 0]) / hyp, -1, 0, 0]])
        return H

    def measurement_model(self, x, landmark_pos):

        px = landmark_pos[0]
        py = landmark_pos[1]
        dist = np.sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)

        Hx = np.array([[dist],
                       [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
        return Hx

    def callback(self, msgs):
        d =100
        if len(msgs.transforms) != 0:
            
            
            for aux in msgs.transforms:
                self.dt=time()-self.time
                msg=aux
                #translation = aux.transform.translation
                # d1 = np.sqrt((translation.z)**2 + (translation.y)**2)
                # if d1<d:
                #     msg=aux
                self.predict()
                translation = msg.transform.translation
                xAruco, yAruco = self.Aruco_location[msg.fiducial_id]
                d = np.sqrt((translation.z)**2 + (translation.y)**2)
                theta = np.arctan2(-translation.y, translation.z)
                Aruco_camera = np.array([d, theta]).T
                Aruco_pos = np.array([xAruco, yAruco]).T/100
                self.update(Aruco_camera, Aruco_pos)
                # self.update(Aruco_camera, HJacobian=self.measurement_jacobian, Hx=self.measurement_model, residual=self.residual,args=(Aruco_pos), hx_args=(Aruco_pos))
                self.time = time()
                self.estimated_x.append(np.ndarray.tolist(self.x[0]))
                self.estimated_y.append(np.ndarray.tolist(self.x[1]))
            self.estimated_ang.append(self.x[2])
            

    def listener(self):
        rospy.Subscriber("/fiducial_transforms",
                         FiducialTransformArray, self.callback)


def main():
    # Number of measurements
    # k=1

    kalman = Kalman_Filter()

    fig, ax = plt.subplots()
    line, = ax.plot([], [], label='Estimated')
    #line2, = ax.plot([], [], label='Predicted')
    x1, y1 = [0, 16.38], [0,0]
    x2, y2 = [1.69,14.03], [1.69,1.69]
    x3,y3=[0,0],[0,16.62]
    x4, y4 = [1.69,1.69], [1.69,14.453]
    x5,y5=[1.69,14.03],[14.453,14.453]
    x6,y6=[0,16.28],[16.62,16.62]
    x7,y7=[16.38,16.38],[0,16.62]
    x8,y8=[14.03,14.03],[1.69,14.453]
    (169,1444.3)
    plt.plot(x1, y1, x2, y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x7,y7,x8,y8,  color='black')
    plt.ion()
    plt.xlabel('X-Position')
    plt.ylabel('Y-Position')
    plt.show()
    while not rospy.is_shutdown():
        # rospy.Subscriber('/camera_info', CameraInfo, kalman.camera_info_callback)
        # rospy.Subscriber("/fiducial_vertices",FiducialArray,kalman.callback1)
        kalman.listener()

        if len(kalman.estimated_x) != 0:
            rospy.loginfo(
                f"\nPosition X:{kalman.estimated_x[-1]}\n Position Y:{kalman.estimated_y[-1]}\n Orientation:{np.rad2deg(kalman.estimated_ang[-1])%360} ")
            line.set_data(kalman.estimated_x, kalman.estimated_y)
            #line2.set_data(kalman.predict_x, kalman.predict_y)
            plt.legend()
            plt.draw()
            plt.pause(0.01)

        kalman.loop.sleep()


if __name__ == "__main__":
    main()
