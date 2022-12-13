function th = invKinBody2Foot(tform, isLeft, varargin)
% This functions expects as input: transform matrix desribing FOOT
% position/orientation w.r.t. body position and orientation
% the inverse kinematics is an analytic solution found in Ali
% et al's 'Closed-Form Inverse Kinematics Joint Solution for Humanoid
% Robots'. To solve the inverse kinematics, Ali sets the base of each leg
% as the end-effector and foot as the base. So we are solving the joint
% angles in reverse order. This way the last three joints (hip pitch, roll,
% yaw) intersect at a point.
%
% Therefore in this function we 
% 1) perform the offset to transform body to foot -> base of each leg to foot.
% 2) find the inverse of the transform to get foot to base of each leg. 
% 3) Find the analytic solution.
%
%     coder.extrinsic('inputParser');
%     p = inputParser;
%     p.addParameter('BendKneeForward', false, @islogical);
%     p.parse(varargin{:});
%
% Copyright 2019 The MathWorks, Inc.

L1 = -0.12;
if isLeft
    L1 = -L1;
end
L2 = 0;
L3 = 0.4;   % Upper leg length
L4 = 0.38;  % Lower leg length
L5 = 0;     % Ankle to foot contact offset

%% 1) Perform some offsets
tform(1,4) = -tform(1,4);
tform = tform - [0 0 0 L1;0 0 0 0;0 0 0 -L2;0 0 0 0];
% Extract position/orientation information
R = tform(1:3,1:3);
p = tform(1:3, 4);

%% 2) Get inverse rotation matrix (in this case, a transpose)
Rp = R';
n = Rp(:,1);
s = Rp(:,2);
a = Rp(:,3);
p = -Rp*p;

%% 3) Compute nalytic solution from the paper
cos4 = ((p(1)+L5)^2 + p(2)^2 + p(3)^2 - L3^2 - L4^2)/(2*L3*L4);
temp = 1 - cos4^2;
if temp < 0
    temp = 0;
    disp('Waning: Unable to reach desired end-effector position/orientation');
end

th4 = atan2(sqrt(temp),cos4);
% NOTE: you can put -sqrt(temp) to change direction of knee bending
temp = (p(1)+L5)^2+p(2)^2;
if temp < 0
    temp = 0;
    disp('Warning: Unable to reach desired end-effector position/orientation');
end
th5 = atan2(-p(3),sqrt(temp))-atan2(sin(th4)*L3,cos(th4)*L3+L4);
th6 = atan2(p(2),-p(1)-L5);
temp = 1-(sin(th6)*a(1)+cos(th6)*a(2))^2;
if temp < 0
    temp = 0;
    disp('Warning: Unable to reach desired end-effector position/orientation');
end
th2 = atan2(-sqrt(temp),sin(th6)*a(1)+cos(6)*a(2));
th2 = th2 + pi/2; % pi/2 offset
th1 = atan2(-sin(th6)*s(1)-cos(th6)*s(2),-sin(th6)*n(1)-cos(th6)*n(2));

th345 = atan2(a(3),cos(th6)*a(1)-sin(th6)*a(2));
th345 = th345 - pi;
th3 = th345 - th4 - th5;

%% Pack the corresponding joint angles
th = [th1 th2 th3 th4 th5 th6];