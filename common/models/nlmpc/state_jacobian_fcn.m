function [A, B] = state_jacobian_fcn(x, u, ref)%,Qp,Rp,Qt,Rt)
% Jacobian of model equations for parking.
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

% Linearize the state equations at the current condition
A = zeros(4,4);
B = zeros(4,2);

A(1,3) = -v*sin(theta);
A(2,3) = v*cos(theta);
A(3,4) = v*(sec(psi)^2)/l;

B(1,1) = cos(theta);
B(2,1) = sin(theta);
B(3,1) = tan(psi)/l;
B(4,2) = 1;

