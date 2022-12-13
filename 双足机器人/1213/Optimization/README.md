# Walking Robots -- Trajectory Optimization
### Copyright 2017-2019 The MathWorks, Inc.

Requires MATLAB R2019b or later.

`optimizeRobotMotion.mlx` is the main optimization script, which has several options listed below
   
a. `parallelFlag` [`true`|`false`]
Use Parallel Computing Toolbox or MATLAB Distributed Computing Server to speed up optimization.
This requires that you have already set up a default parallel computing cluster.

b. `accelFlag` [`true`|`false`]
Run the simulations in Accelerator mode. This will turn off visualization with the Mechanics Explorer, but will run faster.

c. `actuatorType` [`1`|`2`|`3`]
Different fidelity actuators for simulation. Suggest running optimization with `actuatorType = 1` for speed.
* `1`: Ideal motion actuation
* `2`: Torque actuation with PID control
* `3`: Electric motor actuation with average-model PWM, H-Bridge, and PID control

Other files
* `walkingRobotOptim.slx` -- Simulink model configured for optimization
* `simulateWalkingRobot.m` -- Runs a single iteration of the optimization by running the above model with certain parameters, collecting results, and calculating a fitness function
* `doSpeedupTasks.m` -- Configures the optimization problem to use simulation speedup capabilities like Fast Restart, Accelerator mode, and parallel simulation
* `SavedResults` folder -- Contains presaved optimization results
* `createSmoothTrajectory.m` -- Function that generates a cubic polynomial interpolating trajectory from gait waypoints and times

**NOTE:** Once your optimization terminates or you load presaved results, you can view the results using the `compareSimulationTypes.m` script in the `ModelingSimulation` folder.