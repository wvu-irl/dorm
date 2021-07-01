function [nu_Qgrave,iota_R] = get_nuQgrave_iotaR(obj, xprev, u, xnext, ell)
%	Authors: Jared Strader

    %% process determinants
    param_Q = obj.robot_.get_process_noise_covariance(xprev, u);
    param_L = obj.robot_.get_process_noise_jacobian(xprev, u);
    Q_tilde = param_L*param_Q*param_L';
    lambdaQ = max(eig(Q_tilde));
    
    F = obj.robot_.get_process_jacobian(xprev,u);
    lambdaFF = max(eig(F*F'));
    
    nu_Qgrave = det(ell*eye(obj.robot_.x_dim_) + Q_tilde);
    
    %% observation determinants
    param_R = obj.robot_.get_measurement_noise_covariance(xnext);
    param_M = obj.robot_.get_measurement_noise_jacobian(xnext);
    R_tilde = param_M*param_R*param_M';
    
    H = obj.robot_.get_measurement_jacobian(xnext);  
    
    %check if degenerate, and compute iota_R or iota_R_grave
    iota_R=[];
    rankH = rank(H);
    if(rankH < size(H,2))
        [U,S,V] = svd(H);
        H_decomp = H*V';
        H00=H_decomp(:,1:rankH);
        m = size(H,2)-rankH;
        iota_R = (lambdaFF*ell + lambdaQ)^(-m)*det(H00'*inv(R_tilde)*H00); %iota_R_grave
    else
        iota_R = det(H'*inv(R_tilde)*H); %iota_R
    end
end

