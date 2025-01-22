# Quadrotor Aerobatics with Cable-Suspended Payload

<p align="center" width="100%">
<img src="https://github.com/user-attachments/assets/24241262-c516-4b21-b86e-d0c2255c4711" width="400">
</p>

This repository contains the implementation of a Model Predictive Contouring Control (MPCC) approach for performing aerobatic maneuvers with quadrotors carrying cable-suspended payloads.
Our method enables aggressive flight with non-nominal quadrotor orientations while ensuring the _payload swing angle is small_ and the _cable is taut_ throughout the flight.

Using contouring control, we are able to represent complex aerobatic maneuevers such as the Barrel Roll and Power Loop using straight line and circle motion primitives, which are not dynamically feasible 
and cannot be tracked using a typical MPC. 

Additionally, we are able to maximize velocity along the trajectory by introducing a state variable representing progress along the path.

The constrained optimization problem associated with MPCC is solved in real-time using the Real-Time Iteration (RTI) method for sequential quadratic programming (SQP), Levenberg-Marquadt regularization, and Runge-Kutta discretization.


<p align="center" width="100%">
<img src="https://github.com/user-attachments/assets/b498d343-b9f1-45ab-905d-9cd92f965e4f" width="800">
</p>
<p align="center">From left to right: Power Loop, Barrel Roll, Matty Flip</p>

## Dependencies
* [acados](https://github.com/acados/acados)
* [CasADi](https://web.casadi.org/)
* matplotlib
* numpy
* scipy

  
