% 重力和密度参数
density = 500;
foot_density = 1000;
if ~exist('actuatorType','var')
    actuatorType = 1;
end
world_damping = 1e-3;
world_rot_damping = 5e-2;
                    
% 接触力/摩擦力 参数
contact_stiffness = 500;
contact_damping = 50;
mu_k = 0.7;
mu_s = 0.9;
mu_vth = 0.01;
height_plane = 0.025;
plane_x = 25;
plane_y = 10;
contact_point_radius = 1e-4;

% 脚参数
foot_x = 5;
foot_y = 4;
foot_z = 1;
foot_offset = [-1 0 0];

% 腿参数
leg_radius = 0.75;
lower_leg_length = 10;
upper_leg_length = 10;

% 躯干参数
torso_y = 8;
torso_x = 5;
torso_z = 8;
torso_offset_z = -2;
torso_offset_x = -1;
mass = (0.01^3)*torso_y*torso_x*torso_z*density;
g = 9.80665;      

% 铰链参数
joint_damping = 1;
joint_stiffness = 0;
motion_time_constant = 0.01;
joint_limit_stiffness = 1e4;
joint_limit_damping = 10;

%% 强化学习参数
Ts = 0.025; % 采样间隔时间
Tf = 10;    % 整个episode仿真一次的时间
        
% 采取动作值的缩放因子 [-1 1]
max_torque = 3;

% 初始机器人状态
h = 18;     % 胯部高度 [cm]
init_height = foot_z + h + torso_z/2 + torso_offset_z + height_plane/2;
vx0 = 0;    % 初始 X 速度 [m/s]
vy0 = 0;    % 初始 Y 速度 [m/s]
wx0 = 0;    % 初始 X 角速度 [rad/s]
wy0 = 0;    % 初始 Y 角速度 [rad/s]
% 初始机器人脚位置 [m]
leftinit =  [0;0;-h/100];
rightinit = [0;0;-h/100];

% 初始铰链角度
init_angs_L = walkerInvKin(leftinit, upper_leg_length/100, lower_leg_length/100,'3D');     
init_angs_R = walkerInvKin(rightinit, upper_leg_length/100, lower_leg_length/100,'3D');