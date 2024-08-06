import utils
import params
import casadi as ca

def taut_dynamics():
    g = ca.DM([0,0,-9.81]) # Gravity (m/s^2)
    x = ca.SX.sym('x', 22)
    u = ca.SX.sym('Thrust', 4)
    
    P = ca.DM([[-params.dx0, -params.dx1,  params.dx2, params.dx3],
      [params.dy0, -params.dy1, -params.dy2, params.dy3],
     [-params.cT,   params.cT,  -params.cT,  params.cT]]) # Thrust allocation matrix

    # Input Computations
    Torque = P@u # Torque applied by rotors (Nm)
    Thrust = ca.SX.zeros(3)
    Thrust[2] = ca.sum1(u) # Thrust (N)


    # Parsing state vector
    pL = x[0:3] # Load position in inertial frame (m)
    vL = x[3:6] # Load velocity in inertial frame (m/s)
    quat = x[6:10] # Quadrotor quaternion q_WB mapping from body to inertial
    omega = x[10:13] # Quadrotor angular velocity in body frame
    q = x[13:16] # Unit vector from quad to load
    d_q = x[16:19] # Derivative of unit vector from quad to load
    w = x[19:22] # Load angular velocity
    
    # Set up utility Casadi functions
    EP_torqued = utils.EP_torqued()
    quat_to_DCM = utils.quat_to_DCM()

    # Get quadrotor quaternion and angular velocity derivatives
    quatDot, omegaDot = EP_torqued(Torque,quat,omega) # Quaternion KDE
    
    #load attitude dynamics
    DCM = quat_to_DCM(quat)
    qdot = d_q
    d_qdot = (1/(params.mQ*params.l)) * (  ca.cross(q, ca.cross(q,DCM@Thrust)) - params.mQ*params.l*ca.dot(d_q,d_q)@q )
    
    # angular velocity of the load
    wdot = - ca.cross(q,DCM@Thrust) / (params.mQ*params.l)
    
    # quadrotor position dynamics
    pLdot = vL
    vLdot = ( ( ca.dot(q,DCM@Thrust) - params.mQ*params.l*ca.dot(d_q,d_q) )@ q / (params.mQ+params.mL) ) + g

    xdot = ca.vertcat(pLdot, vLdot, quatDot, omegaDot, qdot, d_qdot, wdot)
    
    taut_dynamics = ca.Function('taut_dynamics', [x, u], [xdot])
    
    return taut_dynamics

if __name__ == "__main__":
    taut_dynamics()