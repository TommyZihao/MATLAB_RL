% Simulates the walking robot model with the 3 actuator variants and
% compares results
% Copyright 2017-2019 The MathWorks, Inc.

%% Setup
clc; close all;
mdlName = 'walkingRobot';
bdclose(mdlName)
open_system(mdlName)

%% Simulate
actuatorType = 1;
tic; simout_motion = sim(mdlName,'StopTime','10');
disp(['Compiled and ran Motion actuated simulation in ' num2str(toc) ' seconds']);
actuatorType = 2;
tic; simout_torque = sim(mdlName,'StopTime','10');
disp(['Compiled and ran Torque actuated simulation in ' num2str(toc) ' seconds']);
actuatorType = 3;
tic; simout_motor = sim(mdlName,'StopTime','10');
disp(['Compiled and ran Motor actuated simulation in ' num2str(toc) ' seconds']);

%% Torso Position Plots
figure(1)
subplot(3,1,1)
hold on
plot(get(simout_motion.simout,'measBody').Values.X.Time,get(simout_motion.simout,'measBody').Values.X.Data,'b-');
plot(get(simout_torque.simout,'measBody').Values.X.Time,get(simout_torque.simout,'measBody').Values.X.Data,'r-');
plot(get(simout_motor.simout,'measBody').Values.X.Time, get(simout_motor.simout,'measBody').Values.X.Data, 'k-');
title('Robot Motion')
legend('Motion','Torque','Motor');
title('Simulation Output Comparisons');
xlabel('Time [s]');
ylabel('Torso X Position [m]');
subplot(3,1,2)
hold on
plot(get(simout_motion.simout,'measBody').Values.Y.Time,get(simout_motion.simout,'measBody').Values.Y.Data,'b-');
plot(get(simout_torque.simout,'measBody').Values.Y.Time,get(simout_torque.simout,'measBody').Values.Y.Data,'r-');
plot(get(simout_motor.simout,'measBody').Values.Y.Time, get(simout_motor.simout,'measBody').Values.Y.Data, 'k-');
title('Robot Motion')
legend('Motion','Torque','Motor');
xlabel('Time [s]');
ylabel('Torso Y Position [m]');
subplot(3,1,3)
hold on
plot(get(simout_motion.simout,'measBody').Values.Z.Time,get(simout_motion.simout,'measBody').Values.Z.Data,'b-');
plot(get(simout_torque.simout,'measBody').Values.Z.Time,get(simout_torque.simout,'measBody').Values.Z.Data,'r-');
plot(get(simout_motor.simout,'measBody').Values.Z.Time, get(simout_motor.simout,'measBody').Values.Z.Data, 'k-');
title('Robot Motion')
legend('Motion','Torque','Motor');
xlabel('Time [s]');
ylabel('Torso Z Position [m]');

%% Joint Plots
jointNames = {'ankleroll','anklepitch','knee','hippitch','hiproll','hipyaw'};
for idx = 1:6
    figure(idx+1)
    jName = jointNames{idx};
    
    % Joint angle
    subplot(2,1,1)
    hold on
    modifier = [jName '_angle'];
    plot(get(simout_motion.simout,'measR').Values.(modifier).Time,get(simout_motion.simout,'measR').Values.(modifier).Data,'b-');
    plot(get(simout_torque.simout,'measR').Values.(modifier).Time,get(simout_torque.simout,'measR').Values.(modifier).Data,'r-');
    plot(get(simout_motor.simout,'measR').Values.(modifier).Time, get(simout_motor.simout,'measR').Values.(modifier).Data, 'k-');
    legend('Motion','Torque','Motor');
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    title(jointNames{idx})
        
    % Joint torque
    subplot(2,1,2)
    hold on
    modifier = [jName '_torque'];
    plot(get(simout_motion.simout,'measR').Values.(modifier).Time,get(simout_motion.simout,'measR').Values.(modifier).Data,'b-');
    plot(get(simout_torque.simout,'measR').Values.(modifier).Time,get(simout_torque.simout,'measR').Values.(modifier).Data,'r-');
    plot(get(simout_motor.simout,'measR').Values.(modifier).Time, get(simout_motor.simout,'measR').Values.(modifier).Data, 'k-');
    legend('Motion','Torque','Motor');
    xlabel('Time [s]');
    ylabel('Joint Torque [N*m]');
    ylim(max_torque*[-1, 1])
end


%% Cleanup
bdclose(mdlName)