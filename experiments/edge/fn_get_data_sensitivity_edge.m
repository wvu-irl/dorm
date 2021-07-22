function [data] = fn_get_data_sensitivity_edge(bsp, x, u, P0, ell)

P=P0;
nuP=det(P0);
nuPOptimal=det(P0);
phiP=det(P0);
phiPOptimal=det(P0);
lambdaPBound=max(eig(P0));
lambdaPNotProp=max(eig(P0));
data(1).P = P;
data(1).detP = det(P0);
data(1).nuPOptimal = nuPOptimal;
data(1).phiPOptimal = phiPOptimal;
data(1).lambdaP = max(eig(P0));
data(1).lambdaPBound = lambdaPBound;
data(1).lambdaPNotProp = lambdaPNotProp;
data(1).is_acquired = 0;
for t=1:size(x,2)-1
    %states and controls inputs at time step t
    xt=x(:,t);
    ut=u(:,t);
    xtp1=x(:,t+1);
    
    %previous covariance matrix
    Pprev = P;
%     max_eigenvalue_t = max(eig(Pprev));
    
    %process matrices
    L = bsp.robot_.get_process_noise_jacobian(xt,ut);
    Q = bsp.robot_.get_process_noise_covariance(xt);
    Q_tilde = L*Q*L';
    F = bsp.robot_.get_process_jacobian(xt,ut);
    P = F*P*F' + Q_tilde;
    
    is_acquired = bsp.robot_.is_measurement_acquired(xtp1);
    if(is_acquired)
        %propagate covariance matrix
        M = bsp.robot_.get_measurement_noise_jacobian(xtp1);
        R = bsp.robot_.get_measurement_noise_covariance(xtp1);
        R_tilde = M*R*M';
        H = bsp.robot_.get_measurement_jacobian(xtp1);
        P = inv(inv(P) + H'*inv(R_tilde)*H);
        
        %propagate covariance matrix (using belief dynamics)
%         b = [xt; P_test(:)];
%         b = robot_.propagate_belief_without_noise(b, ut);
%         P_test = reshape(b(robot_.x_dim_+1:end), [robot_.x_dim_,robot_.x_dim_]);
                
        %params for eigenvalue
        lambdaFF = max(eig(F*F'));
        lambdaQ = max(eig(Q_tilde));
        lambdaR = min(eig(H'*inv(R)*H));
        
        %propagate max eigenvalue upper bound
        num = lambdaFF*lambdaPBound + lambdaQ;
        den = lambdaR*(lambdaFF*lambdaPBound + lambdaQ) + 1;
        lambdaPBound = num/den;
        
        %upper bound max eigenvalue
        num = lambdaFF*ell + lambdaQ;
        den = lambdaR*(lambdaFF*ell + lambdaQ) + 1;
        lambdaPNotProp = num/den;

        %nuP optimal alpha
        [nu_Qgrave,iota_R] = bsp.get_nuQgrave_iotaR(xt, ut, xtp1, ell);
        num = nuPOptimal + nu_Qgrave - ell^bsp.robot_.x_dim_;
        den = iota_R*(nuPOptimal + nu_Qgrave - ell^bsp.robot_.x_dim_) + 1;
        nuPOptimal = num/den;
        
        %phi(detP) using optimal alpha
        [nu_Qgrave,iota_R] = bsp.get_nuQgrave_iotaR(xt, ut, xtp1, ell);
        num = det(Pprev) + nu_Qgrave - ell^bsp.robot_.x_dim_;
        den = iota_R*(det(Pprev) + nu_Qgrave - ell^bsp.robot_.x_dim_) + 1;
        phiPOptimal = num/den;
        
    else
        %propagate covariance matrix (using belief dynamics)
%         b = [xt; P_test(:)];
%         b = robot_.propagate_belief_without_noise(b, ut);
%         P_test = reshape(b(robot_.x_dim_+1:end), [robot_.x_dim_,robot_.x_dim_]);

        %eigenvalue parameters
        lambdaFF = max(eig(F*F'));
        lambdaQ = max(eig(Q_tilde));

        %propagate max eigenvalue upper bound
        lambdaPBound = lambdaFF*lambdaPBound + lambdaQ;
        
        %upper bound max eigenvalue
        lambdaPNotProp = lambdaFF*ell + lambdaQ;

        %propagate upper bound from t to t+1 (OPTIMAL ALPHA)
        nu_Qgrave = bsp.get_nuQgrave(x, u, ell);
        nuPOptimal = nuPOptimal + nu_Qgrave - ell^bsp.robot_.x_dim_;
        
        %bound at t+1 using value at t, \phi(\det(P_{t+1}))) optimal alpha
        nu_Qgrave = bsp.get_nuQgrave(x, u, ell);
        phiPOptimal = det(Pprev) + nu_Qgrave - ell^bsp.robot_.x_dim_;
    end
    
    %save data
    data(t+1).P = P;
    data(t+1).detP = det(P);
    data(t+1).nuPOptimal = nuPOptimal;
    data(t+1).phiPOptimal = phiPOptimal;
    data(t+1).lambdaP = max(eig(P));
    data(t+1).lambdaPBound = lambdaPBound;
    data(t+1).lambdaPNotProp = lambdaPNotProp;
    data(t+1).is_acquired = is_acquired;
    data(t+1).x = x(:,t);
    data(t+1).u = u(:,t);
end

end

