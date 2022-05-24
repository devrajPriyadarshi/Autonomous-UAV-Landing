#!/usr/bin/env python

import rospy
import numpy as np
import cvxpy as cp
from numpy.linalg import matrix_power

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from tf.transformations import euler_from_quaternion

class linearMPC():
    
    def __init__(self):

        # rospy.init_node("controller_check_node")
        
        # rospy.Subscriber("odometry", Odometry, self.odomCallback)
        # rospy.Subscriber("command/pose", PoseStamped, self.poseCallback)

        # self.rotorPub = rospy.Publisher( "command/motor_speed", Actuators, queue_size=1)

        #    X =          [x, y, z, | xd, yd, zd, | p, q, r, | pd, qd, rd]
        self.x = np.array([0, 0, 0,    0,   0,  0,  0, 0, 0,    0,  0,  0])

        #    X_dot =          [xd, yd, zd, xdd, ydd, zdd, pd, qd, rd, pdd, qdd, rdd]
        self.x_dot = np.array([ 0,  0,  0,   0,   0,   0,  0,  0,  0,   0,   0,  0])

        #    U =          [T, R, P, Y]
        self.u = np.array([0, 0, 0, 0])

        #    Y =          [x, y, z] ##Here only position is given as reference.
        self.y = np.array([0, 0, 0])

        self.dt = 0.05 # delta tile

        self.g = 9.81 # G

        self.N = 25 # Control Horizon

        self.M = 25 # Prediction Horizon

        self.firstOdomCallback = True # False

        while(self.firstOdomCallback):
            # rospy.loginfo("MPC controller got first call back.")
            self.initParam()
            self.firstOdomCallback = False

    def initParam(self):

        # self.mass = rospy.get_param("~mass")
        # self.Jx = rospy.get_param("~inertial/xx")
        # self.Jy = rospy.get_param("~inertial/yy")
        # self.Jz = rospy.get_param("~inertial/zz")
        # self.I = np.array([ [self.Jx, 0, 0],
        #                     [0, self.Jy, 0],
        #                     [0, 0, self.Jz]])
        
        dt = self.dt
        m = 0.9#self.mass
        g = self.g
        # Jx = self.Jx
        # Jy = self.Jy
        # Jz = self.Jz

        Jx = 1
        Jy = 1
        Jz = 1


        self.A = np.array([ [ 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 1, 0, 0, 0, -g*dt, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 1, 0, +g*dt, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

        self.B = np.array([ [ 0, 0, 0, 0],
                            [ 0, 0, 0, 0],
                            [ 0, 0, 0, 0],
                            [ 0, 0, 0, 0],
                            [ 0, 0, 0, 0],
                            [ dt, 0, 0, 0],
                            [ 0, 0, 0, 0],
                            [ 0, dt/Jx, 0, 0],
                            [ 0, 0, 0, 0],
                            [ 0, 0, dt/Jy, 0],
                            [ 0, 0, 0, 0],
                            [ 0, 0, 0, dt/Jz]])

        self.C = np.array([ [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

        # self.xT = np.array([self.currPos[0], self.currPos[1], self.currPos[2], 0, 0, 0, self.currAtt[0], self.currAtt[1], self.currAtt[2], 0, 0, 0])
        # self.uT = np.array([m*g, 0, 0, 0])

        At = []
        Bt = []
        Ct = []
        
        for i in range(self.M):
            At.append([matrix_power(self.A, i)])

            Btt = []
            Ctt = []
            for j in range(self.M):
                if j >= i:
                    Btt.append(np.zeros((12,4)))
                else:
                    Btt.append(np.matmul(matrix_power(self.A, i-j-1), self.B))
                if i == j:
                    Ctt.append(self.C)
                else:
                    Ctt.append(np.zeros((12, 12)))
            Bt.append(Btt)
            Ct.append(Ctt)


        self.Acap = np.array(np.bmat(At))
        self.Bcap = np.array(np.bmat(Bt))
        self.Ccap = np.array(np.bmat(Ct))

        # print("A = ")
        # print(self.A)
        # print("Shape of A = ", self.A.shape)
        # print("B = ")
        # print(self.B)
        # print("Shape of B = ", self.B.shape)
        # print("C = ")
        # print(self.C)
        # print("Shape of c = ", self.C.shape)
        # print("Acap = ")
        # print(self.Acap)
        # print("Shape of Acap = ", self.Acap.shape)
        # print("Bcap = ")
        # print(self.Bcap)
        # print("Shape of Bcap = ", self.Bcap.shape)
        # print("Ccap = ")
        # print(self.Ccap)
        # print("Shape of Ccap = ", self.Ccap.shape)


    def odomCallback(self, data):
        
        self.firstOdomCallback = True
        self.currPos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.currVel = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.currAtt = np.array(euler_from_quaternion(data.pose.pose.orientation))
        self.currAng = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

    def poseCallback(self, data):

        self.desPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.desAtt = np.array(euler_from_quaternion(data.pose.orientation))

        self.xT = np.array([self.desPos[0], self.desPos[1], self.desPos[2], 0, 0, 0, self.desAtt[0], self.desAtt[1], self.desAtt[2], 0, 0, 0])

    def publish(self):

        A = self.A
        B = self.B
        C = self.C

        

if __name__ == "__main__":
    
    controller = linearMPC()