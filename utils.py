import casadi as ca
import params

def quat_to_DCM():
    q = ca.SX.sym('q', 4)
    dcm = ca.SX.sym('dcm', 3, 3)
    
    # quaternion to DCM (rotation matrix)    
    dcm[0,0] = 1 - 2*(q[1]*q[1] + q[2]*q[2])
    dcm[0,1] = 2*(q[0]*q[1] + q[2]*q[3])
    dcm[0,2] = 2*(q[0]*q[2] - q[1]*q[3])
    dcm[1,0] = 2*(q[0]*q[1] - q[2]*q[3])
    dcm[1,1] = 1 - 2*(q[0]*q[0] + q[2]*q[2])
    dcm[1,2] = 2*(q[1]*q[2] + q[0]*q[3])
    dcm[2,0] = 2*(q[0]*q[2] + q[1]*q[3])
    dcm[2,1] = 2*(q[1]*q[2] - q[0]*q[3])
    dcm[2,2] = 1 - 2*(q[0]*q[0] + q[1]*q[1])
        
    quat_to_dcm = ca.Function('quat_to_dcm', [q], [dcm])
    
    
    return quat_to_dcm

def EP_torqued():
    # Quaternion kinematic differential equations
    # Inputs - I[0], I[1], I[2] (moments of inertia), L (torque), condArr (quaternion
    # and angular velocity)
    # Outputs - quaternion and angular velocity derivatives

    q = ca.SX.sym('q', 4)
    w = ca.SX.sym('w', 3)
    I = params.J
    L = ca.SX.sym('L', 3)
    
    qdot = ca.SX.sym('qdot',4)
    wdot = ca.SX.sym('wdot',3)

    K1 = -1*(I[2]-I[1])/I[0]
    K2 = -1*(I[0]-I[2])/I[1]
    K3 = -1*(I[1]-I[0])/I[2]

    wdot[0] = K1*w[1]*w[2] + L[0]/I[0]
    wdot[1] = K2*w[2]*w[0] + L[1]/I[1]
    wdot[2] = K3*w[0]*w[1] + L[2]/I[2]

    qdot[0:3] = 0.5*(q[3]*w + ca.cross(q[0:3],w))
    qdot[3] = -0.5*ca.dot(q[0:3],w)
    
    EP_torqued = ca.Function('EP_torqued', [L,q,w], [qdot,wdot])
    
    return EP_torqued