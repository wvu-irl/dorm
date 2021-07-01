function [b, b_truth] = propagate_belief_with_noise(obj, b, u, b_truth)
%function [b, b_truth] = propagate_belief_with_noise(b, u, b_truth)
% Propagate the belief b forward a single step given the control inputs
% u, the process model f, and the observation model h.
%
% INPUTS:
% b: [x; P(:)] where x is nx1 vector of states and P is nxn
%      covariance matrix at time step k
% u: [u_1, u_2, ..., u_m]' where u is mx1 vector of control inputs
%      at time step k
% b_truth: [x; P(:)] where x is nx1 vector of states and P is nxn
%          covariance matrix at time step k+1, truth, no noise added
%
% OUTPUTS:
% b: [x; P(:)] where x is nx1 vector of states and P is nxn
%        covariance matrix at time step k+1, estimate, noise added
% b_truth: [x; P(:)] where x is nx1 vector of states and P is nxn
%          covariance matrix at time step k+1, truth, no noise added
%
% Author: Jared Strader
% Structure of this code inspired by some of the code from
% https://github.com/sauravag/bsp-ilqg
%

[tempb, tempb_truth] = update_belief_with_noise(obj, b, u, b_truth);

b=tempb;
b=tempb_truth;

end

function [b_kp1, b_kp1_truth] = update_belief_with_noise(obj, b_k, u_k, b_k_truth)

% check if control input is nan, if not set to zero
if any(isnan(u_k(1,:)))
    u_k = zeros(size(u_k));
%     error('Error! u is nan in belief dynamics');
end

%number of states
n = obj.x_dim_;

%state vector
x_k = b_k(1:n,1);
x_k_truth = b_k_truth(1:n,1);

%covariance matrix
P_k = reshape(b_k(n+1:end), [n,n]);

%propagate state
%technically, the realized motion should not be perfect even though we
%add noise for the estimated motion
x_kp1_truth = f.propagate_without_noise(x_k_truth, u_k, dt);
x_kp1 = f.propagate_with_noise(x_k, u_k, dt);

%proces jacobians
A = f.get_state_transition_jacobian(x_k,u_k,dt);
L = f.get_process_noise_jacobian(x_k,u_k,dt);
Q = f.get_process_noise_covariance(x_k,u_k,dt);
LQL = L*Q*L';

%propagate covariance
P_kp1 = A*P_k*A' + L*Q*L';
% P_prior = P_kp1;

%for most systems, det(F)==1. if this doesn't hold for the system, then
%the uncertainty will grow or shrink without noise in the process
if(det(A) > 1+1e-6 || det(A)< 1-1e-6)
    error('Error: det(F) != 1');
end

%check if measurement is acquired
if(obj.is_measurement_acquired(x_kp1_truth))
    %observation jacobians
    H = f.get_measurement_jacobian(x_kp1);
    M = f.get_measurement_noise_jacobian(x_kp1);
    R = f.get_measurement_noise_covariance(x_kp1);
    HRH = inv(H'*inv(M*R*M')*H);

    %innovation covariance
    S = H*P_kp1*H' + M*R*M';

    %kalman gain
    K = (P_kp1*H')/S;

    %residual
    z_k = f.get_measurement_with_noise(x_kp1_truth); %actual measurement
    y_k = f.get_measurement_without_noise(x_kp1); %expected measurement
    residual =  z_k - y_k;

    %update state and covariance
    x_kp1 = x_kp1 + K*residual;
    P_kp1 = (eye(n) - K*H)*P_kp1;
end

%vectorize updated states and covariance
b_kp1 = zeros(size(b_k));
b_kp1(1:n,1) = x_kp1;
b_kp1(n+1:end,1) = P_kp1(:);

%vectorize updated states and covariance (truth)
b_kp1_truth = b_kp1;
b_kp1_truth(1:n,1) = x_kp1_truth;

%prior belief (i.e., belief before measurement update)
% b_kp1_prior = b_kp1;
% b_kp1_prior(n+1:end,1) = P_prior(:);

end


