import numpy as np
from matplotlib import pyplot as plt

def matty_flip(n_loops, circle_velocity, radius, sample_time):
    circle_center = np.array([5.0, 0.0, -3.5])

    start_phi = np.pi/2 
    end_phi = -(3.0 / 2.0 + 2 * (n_loops - 1)) * np.pi
    circle_traj = generate_vertical_circle(circle_center, 0, radius, circle_velocity, end_phi, start_phi, sample_time)

    # TODO: Add constant heading to circle and exit trajectories
    
    p_0 = np.array([0,0,0])
    v_0 = np.array([0,0,0])
    a_0 = np.array([0,0,0])
    omega = circle_velocity/radius
    sin_phi = np.sin(np.pi/2)
    cos_phi = np.cos(np.pi/2)
    circle_velocity = np.array([-omega*sin_phi, 0, omega*cos_phi])
    circle_acceleration = np.array([-omega**2.*cos_phi, 0, omega**2.*sin_phi])

    enter_traj = generate_spline_desired_position(p_0, v_0, a_0, circle_traj[:,0], 1.1*circle_velocity, circle_acceleration, 5, sample_time)

    p_0 = circle_traj[:,-1]
    v_0 = np.array([-omega*np.sin(end_phi), 0, omega*np.cos(end_phi)])
    a_0 = np.array([-omega**2.*np.cos(end_phi), 0, omega**2.*np.sin(end_phi)])
    end = np.array([0.0, 0.0, -2.0])
    exit_traj = generate_spline_desired_position(p_0, v_0, a_0, end, np.array([0,0,0]), np.array([0,0,0]), 1, sample_time)
    exit_traj = exit_traj[:,1:]


    traj = np.hstack([enter_traj, circle_traj, exit_traj])

    return traj

def generate_vertical_circle(center,orientation,radius,speed,phi_start,phi_end,sampling_time):
    center = np.array(center)
    center = center[:, np.newaxis]
    phi_total = phi_end - phi_start
    direction = phi_total / abs(phi_total)
    omega = direction * abs(speed / radius)
    angle_step = abs(omega * sampling_time)
    d_phi = np.arange(0,abs(phi_total),angle_step)

    phi = phi_start + direction*d_phi
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)

    p = radius*np.array([cos_phi, np.zeros_like(d_phi), -sin_phi]) + center
    v = radius*np.array([-omega*sin_phi, np.zeros_like(d_phi), omega*cos_phi])
    a = radius*np.array([-omega**2.*cos_phi, np.zeros_like(d_phi), omega**2.*sin_phi])
    j = radius*np.array([omega**3.*sin_phi, np.zeros_like(d_phi), omega**3.*cos_phi])
    s = radius*np.array([omega**4.*cos_phi, np.zeros_like(d_phi), -omega**4.*sin_phi])

    # T = 0:1/sampling_freq:abs(phi_total/omega)
    # Z = [p, v, a, j, s]
    # U = generate_nominal_input(T,Z)

    # traj = np.vstack((p, phi))
    # return traj

    return p

def generate_spline_desired_position(p_0, v_0, a_0, p_f,vf,af,t_total,sample_time):
    
    # create a time vector
    T = np.arange(0,t_total,sample_time)
    
    g = np.array([0,0,-9.81])

    # desired future speed and acceleration (QC stops at end of desired traj)
    # vf = np.array([0,30,0])
    # af = np.array([0,0,10])

    # for each axis, compute the change in position/velocity/accel
    Dp = p_f - p_0 - v_0*t_total - 0.5*a_0*t_total**2
    Dv = vf - v_0 - a_0*t_total
    Da = af - a_0

    ax,bx,cx = single_axis_params(Dp[0],Dv[0],Da[0],t_total)
    ay,by,cy = single_axis_params(Dp[1],Dv[1],Da[1],t_total)
    az,bz,cz = single_axis_params(Dp[2],Dv[2],Da[2],t_total)

    # compute the trajectory in each dimension
    # a = [ax,ay,az]
    # b = [bx,by,bz]
    # c = [cx,cy,cz]
    
    px = (ax/120)*np.power(T,5) + (bx/24)*np.power(T,4) + (cx/6)*np.power(T,3) + (a_0[0]/2)*np.power(T,2) + v_0[0]*T + p_0[0]
    py = (ay/120)*np.power(T,5) + (by/24)*np.power(T,4) + (cy/6)*np.power(T,3) + (a_0[1]/2)*np.power(T,2) + v_0[1]*T + p_0[1]
    pz = (az/120)*np.power(T,5) + (bz/24)*np.power(T,4) + (cz/6)*np.power(T,3) + (a_0[2]/2)*np.power(T,2) + v_0[2]*T + p_0[2]

    return np.vstack([px, py, pz])
        
    # Z = make_spline(T,p_0,v_0,a_0,a,b,c)
        
def single_axis_params(Dp,Dv,Da,T):
    M = np.array([[720, -360*T, 60*T**2],
         [-360*T, 168*T**2, -24*T**3],
         [60*T**2, -24*T**3, 3*T**4]])
         
    out = (1/T**5)*M@np.array([Dp,Dv,Da])
    return out[0], out[1], out[2]

# traj = matty_loop(1,4.5,1.5,0.1)
# ax = plt.figure().add_subplot(projection='3d')
# ax.plot(traj[0,:],traj[1,:],traj[2,:])
# ax.set_xlim((-5,5))
# ax.set_ylim((-5,5))
# ax.set_zlim((-5,5))
# ax.set_box_aspect([1,1,1])
# plt.show()