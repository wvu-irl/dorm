function dxdt = state_fcn(x, u, ref)%,Qp,Rp,Qt,Rt)
% State equations for parking: 
% state variables x, y and yaw angle theta.
% control variables v and steering angle delta.

% Copyright 2019 The MathWorks, Inc.

%%
% Parameters
l = 1;

% Variables
theta = x(3);
psi = x(4);
v = u(1);
psi_dot = u(2);

% State equations
% https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html
dxdt = zeros(4,1);
dxdt(1) = v*cos(theta);
dxdt(2) = v*sin(theta);
dxdt(3) = v/l*tan(psi);
dxdt(4) = psi_dot;