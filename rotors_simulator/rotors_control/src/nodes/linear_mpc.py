#!/usr/bin/env python

from math import sin, cos
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

        rospy.init_node("controller_check_node")
        
        rospy.Subscriber("odometry", Odometry, self.odomCallback)
        rospy.Subscriber("command/pose", PoseStamped, self.poseCallback)

        self.rotorPub = rospy.Publisher( "command/motor_speed", Actuators, queue_size=1)

        #    X =          [x, y, z, | xd, yd, zd, | p, q, r, | pd, qd, rd]
        self.x = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

        #    X_dot =          [xd, yd, zd, xdd, ydd, zdd, pd, qd, rd, pdd, qdd, rdd]
        self.x_dot = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

        #    U =          [T, R, P, Y]
        self.u = np.array([[0], [0], [0], [0]])

        #    Y =          [x, y, z] ##Here only position is given as reference.
        # self.y = np.array([[0], [0], [0]])

        self.dt = 0.05 # delta tile

        self.g = 9.81 # G

        self.N = 25 # Control Horizon

        self.M = 25 # Prediction Horizon

        self.firstOdomCallback = False

        while(self.firstOdomCallback):
            # rospy.loginfo("MPC controller got first call back.")
            self.initParam()
            self.firstOdomCallback = False

    def initParam(self):

        self.mass = rospy.get_param("~mass")
        self.Jx = rospy.get_param("~inertial/xx")
        self.Jy = rospy.get_param("~inertial/yy")
        self.Jz = rospy.get_param("~inertial/zz")
        self.I = np.array([ [self.Jx, 0, 0],
                            [0, self.Jy, 0],
                            [0, 0, self.Jz]])
        Kf = []
        Km = []
        ang = []
        L = []
        dir = []

        for i in range(6):
            Rang = "~rotor_configuration/"+ str(i) + "/angle"
            Rdir = "~rotor_configuration/"+ str(i) + "/direction"
            Rlen = "~rotor_configuration/"+ str(i) + "/arm_length"
            RKm = "~rotor_configuration/"+ str(i) + "/rotor_moment_constant"
            RKf = "~rotor_configuration/"+ str(i) + "/rotor_force_constant"
            if rospy.has_param(Rang):
                RangValue = rospy.get_param(Rang)
                RdirValue = rospy.get_param(Rdir)
                RlenValue = rospy.get_param(Rlen)
                RKmValue = rospy.get_param(RKm)
                RKfValue = rospy.get_param(RKf)

                ang.append(RangValue)
                dir.append(RdirValue)
                L.append(RlenValue)
                Km.append(RKmValue)
                Kf.append(RKfValue)
            else:
                self.no_of_rotors = i
        
        inp_r = np.zeros(( 4, self.no_of_rotors))

        self.r_inp = inp_r.T @ np.linalg.inv(inp_r @ inp_r.T)

        for i in range(self.no_of_rotors):
            inp_r[0][i] = Kf[i]
            inp_r[1][i] = Kf[i]*L[i]*sin(ang[i])
            inp_r[2][i] = Kf[i]*L[i]*cos(ang[i])*(-1)
            inp_r[3][i] = Km[i]*dir[i]*(-1)

        dt = self.dt
        m = self.mass
        g = self.g
        Jx = self.Jx
        Jy = self.Jy
        Jz = self.Jz

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

        self.xT = np.array([[self.currPos[0]], [self.currPos[1]], [self.currPos[2]], [0], [0], [0], [self.currAtt[0]], [self.currAtt[1]], [self.currAtt[2]], [0], [0], [0]])
        self.uT = np.array([[m*g], [0], [0], [0]])        

        At = []
        Bt = []
        Ct = []
        uTt = []

        uub = []
        ulb = []
        
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
            uTt.append([self.uT])

            #upper and lower bounds on u
            uub.append([self.uT + np.array( [[10],[3],[3],[3]] ) ])
            ulb.append([self.uT - np.array( [[10],[3],[3],[3]] ) ])



        self.Acap = np.array(np.bmat(At))
        self.Bcap = np.array(np.bmat(Bt))
        self.Ccap = np.array(np.bmat(Ct))
        self.UT= np.array(np.bmat(uTt))

        self.UTub = np.array(np.bmat(uub))
        self.UTlb = np.array(np.bmat(ulb))

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
        # print("UT = ")
        # print(self.UT)
        # print("Shape of UT = ", self.UT.shape)

    def odomCallback(self, data):
        
        self.firstOdomCallback = True
        self.currPos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.currVel = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.currAtt = np.array(euler_from_quaternion(data.pose.pose.orientation))
        self.currAng = np.array([data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z])

        self.x[0][0] = self.currPos[0]
        self.x[0][1] = self.currPos[1]
        self.x[0][2] = self.currPos[2]
        self.x[0][3] = self.currVel[0]
        self.x[0][4] = self.currVel[1]
        self.x[0][5] = self.currVel[2]
        self.x[0][6] = self.currAtt[0]
        self.x[0][7] = self.currAtt[1]
        self.x[0][8] = self.currAtt[2]
        self.x[0][9] = self.currAng[0]
        self.x[0][10] = self.currAng[1]
        self.x[0][11] = self.currAng[2]

    def poseCallback(self, data):

        self.desPos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.desAtt = np.array(euler_from_quaternion(data.pose.orientation))

        self.xT = np.array([[self.desPos[0]], [self.desPos[1]], [self.desPos[2]], [0], [0], [0], [0], [0], [self.desAtt[2]], [0], [0], [0]])

    def publish(self):

        rospy.loginfo("Starting MPC controller")

        dU = cp.Variable(4*self.M, 1)

        UT = cp.Constant(self.UT)
        Acap = cp.Constant(self.Acap)
        Bcap = cp.Constant(self.Bcap)
        Ccap = cp.Constant(self.Ccap)

        Qy = cp.Constant(np.eye(300))
        Qu = cp.Constant(np.eye(100))

        rate = rospy.Rate(20) # 20 Hz

        while(not rospy.is_shutdown()):
            dx = cp.Constant(self.xT - self.x)

            dX = Acap@dx + Bcap@dU
            dY = Ccap@dX

            cost = cp.Minimize( dY.T@Qy@dY + dU.T@Qu@dU + dY[0][-1]*Qy[-1][-1]*dY[0][-1])
            const = [UT + dU >= self.UTlb, UT + dU <= self.UTub]

            prob = cp.Problem(cost, const)

            prob.solve()

            uOptimal = self.uT + dU.value[0][0:3]

            self.cvtRotorSpeed(uOptimal)

            rate.sleep()

    def cvtRotorSpeed(self, u):
        
        Omg_sq = self.r_inp @ u
        Omg = np.sqrt(Omg_sq)

        actuatorMsg = Actuators(angular_velocities=[ Omg[0][0], Omg[0][1], Omg[0][2], Omg[0][3]])

        self.rotorPub(actuatorMsg)        

if __name__ == "__main__":
    
    controller = linearMPC()
    controller.publish()