from casadi import *
import numpy as np
import taut_dynamics, cost_function
import params

T = 0.5 # Time horizon
N = 1 # number of control intervals

# Declare model variables
x = MX.sym('x',22,1)
u = MX.sym('u',4,1)

# Model equations
dynamics = taut_dynamics.taut_dynamics()
xdot = dynamics(x,u)

# Objective term
cost_function = cost_function.cost_function()
L = cost_function(x,u)

# Formulate discrete time dynamics
if False:
   # CVODES from the SUNDIALS suite
   dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
   F = integrator('F', 'cvodes', dae, 0, T/N)
else:
   # Fixed step Runge-Kutta 4 integrator
   M = 4 # RK4 steps per interval
   DT = T/N/M
   f = Function('f', [x, u], [xdot, L])
   X0 = MX.sym('X0', 22)
   U = MX.sym('U', 4)
   X = X0
   Q = 0
   for j in range(M):
       k1, k1_q = f(X, U)
       k2, k2_q = f(X + DT/2 * k1, U)
       k3, k3_q = f(X + DT/2 * k2, U)
       k4, k4_q = f(X + DT * k3, U)
       X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
       Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
   F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

# Evaluate at a test point
Fk = F(x0=[-0.01, 0, -0.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],p=[2,2,2,2])
print(Fk['xf'])
print(Fk['qf'])

# Start with an empty NLP
w=[]
w0 = []
lbw = []
ubw = []
J = 0
g=[]
lbg = []
ubg = []

# "Lift" initial conditions
Xk = MX.sym('X0', 22)
w += [Xk]
lbw += params.initialState.full().flatten().tolist()
ubw += params.initialState.full().flatten().tolist()
w0 += params.initialState.full().flatten().tolist()

# Formulate the NLP
for k in range(N):
    # New NLP variable for the control
    Uk = MX.sym('U_' + str(k), 4)
    w   += [Uk]
    lbw += [0.1, 0.1, 0.1, 0.1]
    ubw += [5, 5, 5, 5]
    w0  += [2, 2, 2, 2]

    # Integrate till the end of the interval
    Fk = F(x0=Xk, p=Uk)
    Xk_end = Fk['xf']
    J=J+Fk['qf']

    # New NLP variable for state at end of interval
    Xk = MX.sym('X_' + str(k+1), 22)
    w   += [Xk]
    lbw += [-inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf]
    ubw += [inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]
    w0  += [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # Add equality constraint
    g   += [Xk_end-Xk]
    lbg += [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ubg += [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob)

# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()

# Plot the solution
x1_opt = w_opt[0::3]
x2_opt = w_opt[1::3]
u_opt = w_opt[2::3]

tgrid = [T/N*k for k in range(N+1)]
# import matplotlib.pyplot as plt
# plt.figure(1)
# plt.clf()
# plt.plot(tgrid, x1_opt, '--')
# plt.plot(tgrid, x2_opt, '-')
# plt.step(tgrid, vertcat(DM.nan(1), u_opt), '-.')
# plt.xlabel('t')
# plt.legend(['x1','x2','u'])
# plt.grid()
# plt.show()