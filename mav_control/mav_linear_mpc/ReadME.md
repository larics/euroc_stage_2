 Linear MPC Controller
=======
 
 Short description
-----------
This controller commands the Firefly attitude to achieve position/velocity reference tracking.

Solver
-----------
It is possible to use FORCES solver http://forces.ethz.ch or CVXGEN http://cvxgen.com/docs/index.html solver.
To switch between solvers, set the appropriate flag in the CMAKE file.

Tuning
-----------
The current default tuning should work fine, for further tuning you can use dynamic reconfiguration GUI.

Simulation vs Real System
-----------
To switch between simulation and real system you need to comment the appropriate line in the CMAKE file and recompile.
In simulation we run a seperate PD attitude controller. 
Launch mav_linear_mpc_sim.launch if you run in simulation or mav_linear_mpc.launch on the real helicopter.


