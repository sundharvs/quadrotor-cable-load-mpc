import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import params

def plot(t, X_true, ref_traj):
    """
    Params:
        t: time values of the discretization
        X_true: array with shape (N_sim, nx)
    """
    pL = X_true[0:3,:]
    q = X_true[13:16,:]
    pQ = pL - params.l * q

    quat = X_true[6:10,:]

    x = quat[0,:]
    y = quat[1,:]
    z = quat[2,:]
    w = quat[3,:]
    R13 = 2*(x*z + y*w)
    R23 = 2*(y*z - x*w)
    R33 = 1 - 2*(x**2 + y**2)
    body_z = np.vstack([R13, R23, R33])

    plt.figure(0)

    plt.subplot(3,1,1)
    plt.plot(t, pQ[0,:], label="actual")
    plt.plot(t, ref_traj[0,:251], label="reference")
    plt.grid(True)
    plt.ylabel('x')
    plt.legend(loc='upper right')

    plt.subplot(3,1,2)
    plt.plot(t, pQ[1,:])
    plt.plot(t, ref_traj[1,:251])
    plt.ylim((-1,1))
    plt.grid(True)
    plt.ylabel('y')

    plt.subplot(3,1,3)
    plt.plot(t, pQ[2,:])
    plt.plot(t, ref_traj[2,:251])
    plt.grid(True)
    plt.ylabel('z')
    plt.xlabel('t [s]')


    plt.figure(1)
    plt.plot(pQ[0,:], pQ[2,:], label="actual")
    plt.plot(ref_traj[0,:], ref_traj[2,:], label="reference")
    plt.quiver(pQ[0,::2], pQ[2,::2], body_z[0,::2], body_z[2,::2])
    plt.gca().set_aspect('equal')
    plt.xlim((-3, 3))
    plt.ylim((-1,4))
    plt.grid(True)
    plt.xlabel('x')
    plt.ylabel('z')
    plt.legend(loc='upper left')

    plt.show()