function penalty = simulateWalkingRobot(params,mdlName,scaleFactor,gait_period,actuatorType)
% Cost function for robot walking optimization
% Copyright 2017-2019 The MathWorks, Inc.

    % Load parameters into function workspace
    robotParameters;
    % Trajectory sample time
    tsTraj = 0.01;          
    % Simulate air drag for stability 
    world_damping = 20;     % Translational damping
    world_rot_damping = 10; % Rotational damping
    
    % Apply variable scaling
    params = scaleFactor*params;
    
    % Extract simulation inputs from parameters
    N = numel(params)/3;
    hip_motion   = deg2rad([params(1:N), params(1)]);
    knee_motion  = deg2rad([params(N+1:2*N), params(N+1)]);
    ankle_motion = deg2rad([params(2*N+1:3*N), params(2*N+1)]);    
    traj_times = linspace(0,gait_period,N+1);
       
    % Evaluate the trajectory at the start and halfway points for right and
    % left legs, respectively, to get initial conditions and trajectory
    % waypoint derivatives
    [q,hip_der,knee_der,ankle_der] = createSmoothTrajectory( ... 
        hip_motion,knee_motion,ankle_motion,gait_period,[0 gait_period/2]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_R = [0 0 -q(1,1) -q(2,1) -q(3,1) 0];
    init_angs_L = [0 0 -q(1,2) -q(2,2) -q(3,2) 0];

    % Simulate the model
    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on');          

    % Unpack logged data
    measBody = get(simout.simout,'measBody').Values;
    xMax = max(abs(measBody.X.Data));
    yEnd = measBody.Y.Data(end);
    tEnd = simout.tout(end);

    % Calculate penalty from logged data
    
    %   Longitudinal (Y) distance traveled without falling 
    %   (ending simulation early) increases reward
    positiveReward = sign(yEnd)*yEnd^2 * tEnd;
    
    %   Lateral (Y) displacement and trajectory aggressiveness 
    %   (number of times the derivative flips signs) decreases reward
    %   NOTE: Set lower limits to prevent divisions by zero
    aggressiveness = 0;
    diffs = [diff(hip_motion) diff(knee_motion) diff(ankle_motion)];
    for idx = 1:numel(diffs)-1
        if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N) 
             aggressiveness = aggressiveness + 1;            
        end
    end
    negativeReward = max(xMax,0.1) * max(aggressiveness,1);
    
    %   Negative sign needed since the optimization minimizes cost function     
    penalty = -positiveReward/negativeReward;        
    
end

