import utils
import params
import casadi as ca

def cost_function():
    x = ca.SX.sym('x', 22)
    u = ca.SX.sym('u', 4)
    quat_to_DCM = utils.quat_to_DCM()
    
    # Compute angle between load and negative quadrotor body z axis
    dcm = quat_to_DCM(x[6:10])
    body_z_negative = -1*dcm[:,2]
    body_z_negative = body_z_negative/ca.norm_2(body_z_negative)
    # x[13:16] = x[13:16]/ca.norm_2(x[13:16])
    load_angle = ca.acos(ca.dot(x[13:16],body_z_negative))

    # Compute State error and cost
    stateError = [3,3,3] - x[0:3] # params.trajectory[0:3,params.timeStep+1]
    cost = 0
    cost = cost + stateError.T@params.Q@stateError
    cost = cost + u.T@params.R@u
    # cost = cost + 1*load_angle
    
    cost_function = ca.Function('cost_function', [x, u], [cost])
    
    return cost_function
