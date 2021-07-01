function [xi, GammaProd] = get_tf_rbrm(obj, x, u)
% function [xi, GammaProd] = get_tf_rbrm(obj, x, u)
% Compute tf for propagating maximum eigenvalue bound at x(:,1) to 
% maximum eigenvalue bound at x(:,end)
%
% Inputs:
% x = n times N+1 matrix where n is the length of the state vector and N 
%     is the number of steps
% u = m times N matrix where n is the length of the control vector and N
%     is the number of steps
%
% For details on computing the bound see "Bopardikar, S. D., Englot, B., 
% & Speranzon, A. (2014). Robust belief roadmap: Planning under uncertain 
% and intermittent sensing. IEEE International Conference on Robotics and 
% Automation, 6122–6129." 
%
% Author: Jared Strader

GammaProd=eye(2);
for t=1:size(x,2)-1
    %states and controls for moving from t to t+1
    xt=x(:,t);
    ut=u(:,t);
    xtp1=x(:,t+1);

    gamma0=[];
    gamma1=[];
    
    %process jacobians
    L = obj.robot_.get_process_noise_jacobian(xt,ut);
    Q = obj.robot_.get_process_noise_covariance(xt);
    Q_tilde = L*Q*L';
    lambdaQ = max(eig(Q_tilde));
    F = obj.robot_.get_process_jacobian(xt,ut);
    lambdaFF = max(eig(F*F'));

    %update tf
    is_acquired=obj.robot_.is_measurement_acquired(xtp1);
    gammaVal=[];
    if(is_acquired)
        %measurement jacobians
        M = obj.robot_.get_measurement_noise_jacobian(xtp1);
        R = obj.robot_.get_measurement_noise_covariance(xtp1);
        R_tilde = M*R*M';
        H = obj.robot_.get_measurement_jacobian(xtp1);
        lambdaR = min(eig(H'*inv(R)*H));
        
        %update tf with measurement
        gammaVal = [lambdaFF, lambdaQ; lambdaR*lambdaFF, lambdaR*lambdaQ+1];
    else
        %update tf without measurement (i.e., H'*inv(R_tilde)*H = 0)
        gammaVal = [lambdaFF, lambdaQ; 0, 1];
    end
    
    GammaProd = gammaVal*GammaProd;
    xi = [GammaProd(1,1); GammaProd(1,2); GammaProd(2,1); GammaProd(2,2)];
end

end

