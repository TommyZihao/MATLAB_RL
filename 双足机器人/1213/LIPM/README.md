# Walking Robots -- Linear Inverted Pendulum Model (LIPM)
### Copyright 2019 The MathWorks, Inc.

Contains files for linear inverted pendulum model based control of the walking robot simulation.

Requires MATLAB R2019b or later.

## PROCESS FOR CREATING TRAJECTORY 
By running the scripts one by one, we create the body's trajectory using the LIPM, then we convert the body's trajectory into joint trajectory.  

1. `applicationLIPM.mlapp` -- Interactive application to understand how the model behaves  
2. `animateLIPM.mlx` -- Create symmetric walking pattern based on the model
3. `animateLIPMLocal.mlx` -- Transform the body trajectory to feet trajectory 
4. `animateInverseKinematics.mlx` -- Calculate joint trajectory from feet trajectory 
5. `walkingRobotLIPM.slx` -- Run simulation based on feet trajectory 

## MODEL FILES 
This robot has 6 degrees of freedom per leg. The kinematic structure is based on HUBO. 

* `walkingRobotLIPM.slx` -- Run simulation based on foot trajectory 
* `interactiveRobotLIPM.slx` -- Interactively control foot positions to test inverse kinematics and stability

## COMMON FILES
* `getSwingFootTraj.m` -- Get swing foot trajectory based on cubic polynomial trajectory
* `invKinBody2Foot.m` -- Calculate joint angles for foot position and orientation
* `defaultstepinfos.mat` -- Variables for running the second script without running the first 
* `defaultfootinfos.mat` -- Variables for running the third script without running the second
* `defaultsimulationinputs.mat` -- Variables for running the Simulink Model without running the scripts 

