function nlobj = create_nlmpc_ackermann_obj(dt)
    x_dim = 4; %x and y position, heading, steering angle
    u_dim = 2; %velocity, turning rate
    y_dim = 4; %x, y, heading, steering angle

    %probably not needed
%     Qp_ = diag([0.1 0.1 0]);
%     Rp_ = 0.01*eye(2);
%     Qt_ = diag([1 5 100]); 
%     Rt_ = 0.1*eye(2);

    mpcN = 100;
    predict_horizon = 20;
    control_horizon = 20;

	mpcverbosity('off');
    nlobj = nlmpc(x_dim,y_dim,u_dim);
   
    nlobj.Ts = dt;
    nlobj.PredictionHorizon = predict_horizon;
    nlobj.ControlHorizon = control_horizon;
    
    nlobj.MV(1).Min = 0;
    nlobj.MV(1).Max = 1;
    nlobj.MV(2).Min = -pi/2;
    nlobj.MV(2).Max = pi/2;

    nlobj.Model.StateFcn = "state_fcn";
    nlobj.Jacobian.StateFcn = "state_jacobian_fcn";

    % nlobj_.Optimization.CustomCostFcn = "cost_fcn";
    % nlobj_.Optimization.ReplaceStandardCost = true;
    % nlobj_.Jacobian.CustomCostFcn = "cost_jacobian_fcn";

    % nlobj_.Optimization.CustomIneqConFcn = "inequality_constraints_fcn";
    % nlobj_.Jacobian.CustomIneqConFcn = "inequality_constraints_jacobian_fcn";

    nlobj.Optimization.SolverOptions.FunctionTolerance = 0.01;
    nlobj.Optimization.SolverOptions.StepTolerance = 0.01;
    nlobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;
    nlobj.Optimization.SolverOptions.OptimalityTolerance = 0.01;
    nlobj.Optimization.SolverOptions.MaxIter = mpcN;

    % opt = nlmpcmoveopt;
    % opt.X0 = [linspace(x0(1),goal(1),predict_horizon)', ...
    %           linspace(x0(2),goal(2),predict_horizon)', ...
    %           linspace(x0(3),goal(3),predict_horizon)', ...
    %           linspace(x0(4),goal(4),predict_horizon)'];
    % opt.MV0 = zeros(predict_horizon,nu);

    %paras = {goal,Qp,Rp,Qt,Rt}';
    % paras = goal;
    % nlobj.Model.NumberOfParameters = 1;%numel(paras);
    % opt.Parameters = paras;
    
end