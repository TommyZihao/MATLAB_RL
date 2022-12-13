function [q, qd, qdd] = getSwingFootTraj(footpos0, footpos1, swingheight, timestp0, timestpf, Ts)
% Returns cubic polynomial trajectory for the robot's swing foot
%
% Copyright 2019 The MathWorks, Inc.

% Trajectory for X and Y
waypointsXY = [footpos0(1:2) footpos1(1:2)];
timestampsXY = [timestp0 timestpf];
timevecswingXY = timestampsXY(1):Ts:timestampsXY(end);
[XYq, XYqd, XYqdd, ~] = cubicpolytraj(waypointsXY, timestampsXY, timevecswingXY);

% Trajectory for Z
waypointsZ = [footpos0(3) footpos0(3)+swingheight footpos0(3)];
timestpMid = (timestp0+timestpf)/2; % Top of swing at midpoint
timestampsZ = [timestp0 timestpMid timestpf];
timevecswingZ = timestampsZ(1):Ts:timestampsZ(end);
[Zq, Zqd, Zqdd, ~] = cubicpolytraj(waypointsZ, timestampsZ, timevecswingZ);

% combine xy and z trajectory
q = [XYq; Zq];
qd = [XYqd; Zqd];
qdd = [XYqdd; Zqdd];

end