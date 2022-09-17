function [G,Gmv,Ge] = cost_jacobian_fcn(X,U,e,data,ref)%,Qp,Rp,Qt,Rt)
% Jacobian of cost function for parking.

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
    G = zeros(p,data.NumOfStates);
    Gmv = zeros(p,length(data.MVIndex));
    Ge = 0;
    for i=1:p
        % Running cost Jacobian
        G(i,:) = 2 * (X(i+1,:)-data.References(i,:)) * Qp;
        Gmv(i,:) = 2 * U(i,:) * Rp;
    end
    G(p,:) = G(p,:) + 2 * (X(p+1,:)-data.References(i,:)) * Qt; 
    Gmv(p,:) = Gmv(p,:) + 2 * U(p,:) * Rt;
    
end

