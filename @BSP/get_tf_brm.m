function [xi] = get_tf_brm(obj, x, u)
% function [xi] = get_tf_brm(obj, x, u)
% Compute tf for transforming covariance matrix at x(:,1) to covariance
% matrix as x(:,end)
%
% Inputs:
% x = n times N+1 matrix where n is the length of the state vector and N 
%     is the number of steps
% u = m times N matrix where n is the length of the control vector and N
%     is the number of steps
%
% For details on computing the transfer function see "Prentice, S., & Roy, 
% N. (2009). The belief roadmap: Efficient planning in belief space by 
% factoring the covariance. International Journal of Robotics Research, 
% 32(11), 1231–1237." 
%
% Author: Jared Strader

gamma=eye(2*obj.robot_.x_dim_);
Z = zeros(obj.robot_.x_dim_,obj.robot_.x_dim_);
I = eye(obj.robot_.x_dim_);
for t=1:size(x,2)-1
    %states and controls for moving from t to t+1
    xt=x(:,t);
    ut=u(:,t);
    xtp1=x(:,t+1);

    %process jacobians
    L = obj.robot_.get_process_noise_jacobian(xt,ut);
    Q = obj.robot_.get_process_noise_covariance(xt);
    Q_tilde = L*Q*L';
    F = obj.robot_.get_process_jacobian(xt,ut);

    %update tf
    is_acquired=obj.robot_.is_measurement_acquired(xtp1);
    if(is_acquired)
        %measurement jacobians
        M = obj.robot_.get_measurement_noise_jacobian(xtp1);
        R = obj.robot_.get_measurement_noise_covariance(xtp1);
        R_tilde = M*R*M';
        H = obj.robot_.get_measurement_jacobian(xtp1);

        %update tf with measurement
        M1 = [Z, I; I, H'*inv(R_tilde)*H];
        M2 = [Z, inv(F'); F, Q_tilde*inv(F')];
    else
        %update tf without measurement (i.e., H'*inv(R_tilde)*H = 0)
        M1 = [Z, I; I, Z];
        M2 = [Z, inv(F'); F, Q_tilde*inv(F')];
    end
    
    gamma = M1*M2*gamma;
end

xi = gamma(:);

end

