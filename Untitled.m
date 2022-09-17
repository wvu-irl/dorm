clear all; clc;
close all;     

x_dim_ = 4; %x and y position, heading, steering angle
u_dim_ = 2; %velocity, turning rate
w_dim_ = 2; %control noise
y_dim_ = 4; %x, y, heading, steering angle
v_dim_ = 4; %measurement noise        

Qp_ = diag([0.1 0.1 0]);
Rp_ = 0.01*eye(2);
Qt_ = diag([1 5 100]); 
Rt_ = 0.1*eye(2);
N_ = 100;
mpcN_ = 40;
predict_horizon_ = 10;
control_horizon_ = 10;

%         mpcverbosity('off');
nlobj_ = nlmpc(x_dim_,x_dim_,u_dim_);
nlobj_.MV(1).Min = 0;
nlobj_.MV(1).Max = 1;
nlobj_.MV(2).Min = -pi/4;
nlobj_.MV(2).Max = pi/4;

nlobj_.Model.StateFcn = "state_fcn";
nlobj_.Jacobian.StateFcn = "state_jacobian_fcn";

% nlobj_.Optimization.CustomCostFcn = "cost_fcn";
% nlobj_.Optimization.ReplaceStandardCost = true;
% nlobj_.Jacobian.CustomCostFcn = "cost_jacobian_fcn";

% nlobj_.Optimization.CustomIneqConFcn = "inequality_constraints_fcn";
% nlobj_.Jacobian.CustomIneqConFcn = "inequality_constraints_jacobian_fcn";

nlobj_.Optimization.SolverOptions.FunctionTolerance = 0.01;
nlobj_.Optimization.SolverOptions.StepTolerance = 0.01;
nlobj_.Optimization.SolverOptions.ConstraintTolerance = 0.01;
nlobj_.Optimization.SolverOptions.OptimalityTolerance = 0.01;
nlobj_.Optimization.SolverOptions.MaxIter = mpcN_;