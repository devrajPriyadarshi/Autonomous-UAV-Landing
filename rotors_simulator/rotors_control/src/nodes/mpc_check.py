
import cvxpy as cp
import numpy as np
from numpy.linalg import matrix_power

import matplotlib.pyplot as plt


M = 100

dx = np.array([ [0],
                [0],
                [-4],
                [0],
                [0],
                [0],
                [-0.2],
                [0.02],
                [0],
                [0],
                [0],
                [0]])

dt = 0.05
m = 0.716
g = 9.8
Jx = 0.007
Jy = 0.007
Jz = 0.012

A = np.array([  [ 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                [ 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
                [ 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                [ 0, 0, 0, 1, 0, 0, 0, g*dt, 0, 0, 0, 0],
                [ 0, 0, 0, 0, 1, 0, -g*dt, 0, 0, 0, 0, 0],
                [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [ 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
                [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
                [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
                [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

B = np.array([  [ 0, 0, 0, 0],
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

C = np.array([  [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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

MG = np.zeros((12,1))
MG[5][0] = -1*abs(g)*dt
print("MG = ", MG)

At = []
Bt = []
Ct = []
uTt = []

uub = []
ulb = []
mgt = []

for i in range( M):
    At.append([matrix_power( A, i)])

    Btt = []
    Ctt = []
    for j in range( M):
        if j >= i:
            Btt.append(np.zeros((12,4)))
        else:
            Btt.append(np.matmul(matrix_power( A, i-j-1),  B))
        if i == j:
            Ctt.append( C)
        else:
            Ctt.append(np.zeros((12, 12)))
    Bt.append(Btt)
    Ct.append(Ctt)

    uub.append([np.array( [[10],[10],[10],[10]] ) ])
    ulb.append([-1*np.array( [[10],[10],[10],[10]] ) ])
    mgt.append([MG ])

Acap = np.array(np.bmat(At))
Bcap = np.array(np.bmat(Bt))
Ccap = np.array(np.bmat(Ct))
dUub = np.array(np.bmat(uub))
dUlb = np.array(np.bmat(ulb))
MGcap = np.array(np.bmat(mgt))

print(Acap, Acap.shape)
print(Bcap, Bcap.shape)
print(Ccap, Ccap.shape)

dU = cp.Variable(4*M, 1)

dX = Acap@dx + Bcap@dU #+ MGcap
dY = Ccap@dX

Qy = np.eye(12*M)
Qu = np.eye(4*M)
# cost = cp.Minimize( dY.T@Qy@dY + dU.T@Qu@dU )
cost = cp.Minimize( 1*cp.quad_form(dY, Qy) + 1*cp.quad_form(dU, Qu) )
# cost = cp.Minimize( 100*cp.norm(dY) + 0.1*cp.norm(dU) )
const = [dUlb <= dU , dU <= dUub]
# const = []

prob = cp.Problem(cost, const)
# print("Prob is DCP? :", prob.is_dcp())

result = prob.solve()

dUop = np.array(dU.value)
# print("dUop = ", dUop)

print("dU = ", [[dUop[0][0]], [dUop[1][0]], [dUop[2][0]], [dUop[3][0]]])
print("result = ", result)
print("dY = ", (Ccap @ (Acap @dx  + Bcap @ dU.value))[12*M - 10][0])

V = np.array(Ccap @ (Acap @dx  + Bcap @ ( dU.value) ))

z = []
zd = []

for i in range(6, 12*M,12):
    z.append(V[i][0])
    # zd.append(V[i+3][0])

plt.plot(z)
plt.show()