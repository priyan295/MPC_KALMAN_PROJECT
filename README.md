# MPC_KALMAN_PROJECT
In this project, we were given an industrial air-dryer dataset. The accurate description of the underlying dynamical system was provided along with the parameter values pertaining to the nonlinear system. 
We linearized the system to and run a Kalman filter to estimate the states.
Subsequently, with the predicted values of the Kalman filter, we proposed the control action by MPC to regulate the output around the reference value.
The prediction and control horizon were tuned through a trial-error method.
Stability of the Linearised system was ensured as the first step
Initially, we proposed an unconstrained MPC therefore avoiding the issue of recursive feasibility.
