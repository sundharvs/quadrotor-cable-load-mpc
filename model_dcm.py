from acados_template import AcadosModel
import casadi as ca
import params
import utils

def export_model() -> AcadosModel:

    model_name = 'quad_cable_ode'

    P = ca.DM([[-params.dx0, -params.dx1,  params.dx2, params.dx3],
    [params.dy0, -params.dy1, -params.dy2, params.dy3],
    [-params.cT,   params.cT,  -params.cT,  params.cT]]) # Thrust allocation matrix

    g = ca.DM([0,0,-9.81]) # Gravity (m/s^2)

    # set up states & controls
    x = ca.SX.sym('x', 27)
    u = ca.SX.sym('Thrust', 4)

    # Input Computations
    Torque = P@u # Torque applied by rotors (Nm)
    Thrust = ca.SX.zeros(3)
    Thrust[2] = ca.sum1(u) # Thrust (N)

    # Parsing state vector
    pL = x[0:3] # Load position in inertial frame (m)
    vL = x[3:6] # Load velocity in inertial frame (m/s)
    R = x[6:15] # Quadrotor quaternion q_WB mapping from body to inertial
    omega = x[15:18] # Quadrotor angular velocity in body frame
    q = x[18:21] # Unit vector from quad to load
    d_q = x[21:24] # Derivative of unit vector from quad to load
    w = x[24:27] # Load angular velocity

    # xdot
    xdot = ca.SX.sym('xdot', 27)

    # dynamics

    # Get quadrotor quaternion and angular velocity derivatives
    skew_omega = ca.SX.zeros(3, 3)
    skew_omega[0, 1] = -omega[2]
    skew_omega[0, 2] = omega[1]
    skew_omega[1, 0] = omega[2]
    skew_omega[1, 2] = -omega[0]
    skew_omega[2, 0] = -omega[1]
    skew_omega[2, 1] = omega[0]

    DCM = ca.reshape(R,(3,3))

    Rdot = ca.reshape(DCM @ skew_omega, (9,1))
    omegaDot = 1./params.J * ( Torque - ca.cross(omega, params.J * omega) )
    
    #load attitude dynamics
    qdot = d_q
    d_qdot = (1/(params.mQ*params.l)) * (  ca.cross(q, ca.cross(q,DCM@Thrust)) - params.mQ*params.l*ca.dot(d_q,d_q)@q )
    
    # angular velocity of the load
    wdot = - ca.cross(q,DCM@Thrust) / (params.mQ*params.l)
    
    # quadrotor position dynamics
    pLdot = vL
    vLdot = ( ( ca.dot(q,DCM@Thrust) - params.mQ*params.l*ca.dot(d_q,d_q) )@ q / (params.mQ+params.mL) ) + g

    f_expl = ca.vertcat(pLdot, vLdot, Rdot, omegaDot, qdot, d_qdot, wdot)
    f_impl = xdot - f_expl

    # model nonlinear cost term (we're going from load position to quadrotor position here)
    pQ = pL - params.l * q
    vQ = vL - params.l * ca.cross(w,q)

    # model nonlinear constraint (load angle)
    body_z_negative = -1*DCM[:,2]
    load_angle = ca.acos(ca.dot(q,body_z_negative)/(ca.norm_2(body_z_negative)*ca.norm_2(q)))

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name
    model.cost_y_expr = ca.vertcat(pQ, load_angle)
    model.cost_y_expr_e = ca.vertcat(pQ, load_angle)

    # store meta information
    # model.x_labels = ['$x$ [m]', r'$\theta$ [rad]', '$v$ [m]', r'$\dot{\theta}$ [rad/s]']
    # model.u_labels = ['$F$']
    # model.t_label = '$t$ [s]'

    return model

