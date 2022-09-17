function cost =  cost_fcn(X,U,e,data,ref)%,Qp,Rp,Qt,Rt)
% Cost function for parking.

% Copyright 2019 The MathWorks, Inc.
    Qp = zeros(4,4);
    Qp(1,1) = 2;
    Qp(2,2) = 2;
    Qt = zeros(4,4);
    Qt(1,1) = 2;
    Qt(2,2) = 2;
    Rp = zeros(2,2);
    Rt = zeros(2,2);
    p = data.PredictionHorizon;
    
    % process cost
    cost = 0;
    for idx = 1:p
        runningCost = (X(idx+1,:)-data.References(idx,:))*Qp*(X(idx+1,:)-data.References(idx,:))' + U(idx,:)*Rp*U(idx,:)';
        cost = cost + runningCost;
    end
    % terminal cost
    terminal_cost = (X(p+1,:)-data.References(idx,:))*Qt*(X(p+1,:)-data.References(idx,:))' + U(p,:)*Rt*U(p,:)';
    % total cost
    cost = cost + terminal_cost;
end