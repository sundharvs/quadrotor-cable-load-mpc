import numpy as np
import matplotlib.pyplot as plt
import params
from utils import quat_to_DCM
from scipy.io import savemat

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
    DCM = quat_to_DCM()

    load_angle = np.zeros(np.size(quat,1))
    body_z = np.zeros((3,np.size(quat,1)))
    dcm_hist = np.zeros((9,np.size(quat,1)))
    for i in range(np.size(quat,1)):
        dcm = DCM(quat[:,i])
        dcm_hist[:,i] = np.reshape(dcm,(9,))
        body_z[:,i] = np.ravel(dcm[:,2])
        load_angle[i] = np.arccos(np.dot(q[:,i],-1*body_z[:,i])/(np.linalg.norm(-1*body_z[:,i])*np.linalg.norm(q[:,i])))

    anim_traj = np.vstack([pQ, pL, dcm_hist])
    savemat('trajectory.mat', {'traj': anim_traj})

    plt.figure(0)

    plt.subplot(3,1,1)
    plt.plot(t, pQ[0,:], label="actual")
    plt.plot(t, ref_traj[0,:-24], label="reference")
    plt.grid(True)
    plt.ylabel('x')
    plt.legend(loc='upper right')

    plt.subplot(3,1,2)
    plt.plot(t, pQ[1,:])
    plt.plot(t, ref_traj[1,:-24])
    plt.ylim((-1,1))
    plt.grid(True)
    plt.ylabel('y')

    plt.subplot(3,1,3)
    plt.plot(t, pQ[2,:])
    plt.plot(t, ref_traj[2,:-24])
    plt.grid(True)
    plt.ylabel('z')
    plt.xlabel('t [s]')
    plt.ylim((-5,5))


    plt.figure(1)
    plt.plot(pQ[0,:], pQ[2,:], label="actual")
    plt.plot(ref_traj[0,:], ref_traj[2,:], label="reference")
    plt.quiver(pQ[0,::2], pQ[2,::2], body_z[0,::2], body_z[2,::2])
    plt.gca().set_aspect('equal')
    plt.xlim((-2, 8))
    plt.ylim((-5, 2))
    plt.grid(True)
    plt.xlabel('x')
    plt.ylabel('z')
    plt.legend(loc='upper left')

    plt.figure(2)
    plt.plot(t, np.rad2deg(load_angle))
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Load angle [deg]')

    plt.show()