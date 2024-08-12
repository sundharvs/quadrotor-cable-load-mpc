from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from model import export_model
from utils import generate_vertical_circle_traj
from plotting import plot
import numpy as np
import scipy.linalg
from casadi import vertcat
import params

def setup(x0, Fmax, N_horizon, Tf, RTI=False):
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_model()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = 3
    ny_e = 3

    ocp.dims.N = N_horizon

    # set cost module
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # Q_mat = np.diag([400, 2, 150])
    Q_mat = np.diag([160000, 0.001, 120000])
    # R_mat = np.diag([0.2, 0.2, 0.2, 0.2])

    # ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W = Q_mat
    ocp.cost.W_e = 50*Q_mat

    # ocp.model.cost_y_expr = model.cost_y_expr
    # ocp.model.cost_y_expr_e = model.cost_y_expr_e
    ocp.cost.yref  = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((ny_e, ))

    # set constraints
    ocp.constraints.lbu = np.array([0, 0, 0, 0])
    ocp.constraints.ubu = np.array([5, 5, 5, 5])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.constraints.x0 = x0

    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES' # PARTIAL_CONDENSING_HPIPM
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.sim_method_newton_iter = 10
    ocp.solver_options.levenberg_marquardt = 14.5

    if RTI:
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    else:
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING' # turns on globalization
        ocp.solver_options.nlp_solver_max_iter = 1000

    ocp.solver_options.qp_solver_cond_N = N_horizon

    # set prediction horizon
    ocp.solver_options.tf = Tf

    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)

    # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = solver_json)

    return acados_ocp_solver, acados_integrator


def main(use_RTI=False):

    x0 = params.initialState
    Fmax = 10

    Tf = 0.5 # prediction horizon
    N_horizon = 25 # number of steps in horizon

    ocp_solver, integrator = setup(x0, Fmax, N_horizon, Tf, use_RTI)

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    Nsim = 250
    simX = np.zeros((nx, Nsim+1))
    simU = np.zeros((nu, Nsim))

    simX[:,0] = x0

    # set up reference trajectory
    ref_traj = np.zeros((3,Nsim+N_horizon))
    circle_traj = generate_vertical_circle_traj([0,0,1.5], 0, 1.5, 4.5, np.pi/2, -3/2*np.pi, N_horizon/Tf)
    ref_traj[:,:min(Nsim, circle_traj.shape[1])] = circle_traj[:,:min(Nsim, circle_traj.shape[1])]
    # ref_traj[0:2:, :] = np.vstack([np.linspace(0,3,Nsim+N_horizon), np.linspace(0,3,Nsim+N_horizon)])

    if use_RTI:
        t_preparation = np.zeros((Nsim))
        t_feedback = np.zeros((Nsim))

    else:
        t = np.zeros((Nsim))

    # do some initial iterations to start with a good initial guess
    num_iter_initial = 5
    for _ in range(num_iter_initial):
        ocp_solver.solve_for_x0(x0_bar = x0)

    # closed loop
    for i in range(Nsim):

        if use_RTI:
            # preparation phase
            ocp_solver.options_set('rti_phase', 1)
            status = ocp_solver.solve()
            t_preparation[i] = ocp_solver.get_stats('time_tot')

            # set initial state
            ocp_solver.set(0, "lbx", simX[:, i])
            ocp_solver.set(0, "ubx", simX[:, i])

            # set reference trajectory
            for j in range(N_horizon):
                yref = ref_traj[:,i+j]
                ocp_solver.set(j, "yref", yref)

            yref_N = ref_traj[0:nx,i+j] # only states
            ocp_solver.cost_set(N_horizon, 'yref', yref_N)

            # feedback phase
            ocp_solver.options_set('rti_phase', 2)
            status = ocp_solver.solve()
            t_feedback[i] = ocp_solver.get_stats('time_tot')

            simU[:, i] = ocp_solver.get(0, "u")

        else:
            # solve ocp and get next control input
            simU[:,i] = ocp_solver.solve_for_x0(x0_bar = simX[:, i])

            t[i] = ocp_solver.get_stats('time_tot')

        # simulate system
        simX[:, i+1] = integrator.simulate(x=simX[:, i], u=simU[:,i])

    # evaluate timings
    if use_RTI:
        # scale to milliseconds
        t_preparation *= 1000
        t_feedback *= 1000
        print(f'Computation time in preparation phase in ms: \
                min {np.min(t_preparation):.3f} median {np.median(t_preparation):.3f} max {np.max(t_preparation):.3f}')
        print(f'Computation time in feedback phase in ms:    \
                min {np.min(t_feedback):.3f} median {np.median(t_feedback):.3f} max {np.max(t_feedback):.3f}')
    else:
        # scale to milliseconds
        t *= 1000
        print(f'Computation time in ms: min {np.min(t):.3f} median {np.median(t):.3f} max {np.max(t):.3f}')

    # plot results
    model = ocp_solver.acados_ocp.model
    plot(np.linspace(0, (Tf/N_horizon)*Nsim, Nsim+1), simX, ref_traj)

    ocp_solver = None


if __name__ == '__main__':
    # main(use_RTI=False)
    main(use_RTI=True)
