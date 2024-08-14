import casadi as ca
import numpy as np

# Geometry
dx0 = 0.19  # X distance of first rotor  (m)
dx1 = 0.19  # X distance of second rotor (m)
dx2 = 0.19  # X distance of third rotor  (m)
dx3 = 0.19  # X distance of fourth rotor (m)

dy0 = 0.19  # Y distance of first rotor  (m)
dy1 = 0.19  # Y distance of second rotor (m)
dy2 = 0.19  # Y distance of third rotor  (m)
dy3 = 0.19  # Y distance of fourth rotor (m)

# drag
cT = 3.75e-9  #0.05  # Rotor drag torque constant (idk)

# Mass parameters
mQ = 0.547  # mass of quadrotor (kg)
J = ca.DM([0.0033, 0.0033, 0.0033])
maxThrust = 5  # maximum thrust (N)
minThrust = 0  # minimum thrust (N)
mL = 0.05  # mass of payload (kg)
l = 1.5  # length of cable [m]

# MPC Parameters
Q = 40*ca.DM_eye(3) 
R = 0.01*ca.DM_eye(4) 
predHorizon = 8 
dt = 0.05 
equalityTolerance = 0 

# Optimizer Parameters
tolerance = 0.1 

# Plot?
plot = True 

initialState = np.array([-0.01, 0, -0.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
# initialStateDCM = np.array([-0.01, 0, -0.3, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])


# sets initial load position to be l (cable length) away from quadrotor
if np.linalg.norm(initialState[0:3]) != 0:
    initialState[0:3] = l*initialState[0:3] / np.linalg.norm(initialState[0:3])


# q (unit vector between quadrotor and load) initial condition
initialState[13:16] = (initialState[0:3] - [0,0,0]) / l
