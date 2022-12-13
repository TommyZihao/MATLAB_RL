function [q,hip_der,knee_der,ankle_der] = createSmoothTrajectory(hip,knee,ankle,period,evalTimes)
% Generates cubic spline trajectory parameters given waypoints
% for 3 joints (ankle, knee, hip) and total gait period
% Copyright 2017-2019 The MathWorks, Inc.

% Create necessary values for calculations
numPoints = numel(hip);
traj_times = linspace(0,period,numPoints);

% Calculate derivatives
dt = period/(numPoints-1);
hip_der = [0, 0.5*( diff(hip(1:end-1)) + diff(hip(2:end)) )/dt, 0];
knee_der = [0, 0.5*( diff(knee(1:end-1)) + diff(knee(2:end)) )/dt, 0];
ankle_der = [0, 0.5*( diff(ankle(1:end-1)) + diff(ankle(2:end)) )/dt, 0];

% Set initial conditions
% Evaluate the trajectory at the start and halfway points for right and
% left legs, respectively
q = cubicpolytraj([hip;knee;ankle],traj_times,evalTimes,...
    'VelocityBoundaryCondition',[hip_der;knee_der;ankle_der]);
