function [nu_Qgrave] = get_nuQgrave(obj, x, u, ell)
%	Authors: Jared Strader

    param_Q = obj.robot_.get_process_noise_covariance(x, u);
    param_L = obj.robot_.get_process_noise_jacobian(x, u);
    Q_tilde = param_L*param_Q*param_L';
    
    nu_Qgrave = det(ell*eye(obj.robot_.x_dim_) + Q_tilde);
end

