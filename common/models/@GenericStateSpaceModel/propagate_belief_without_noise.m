function [b] = propagate_belief_without_noise(obj, b, u)
%function [b] = propagate_belief_without_noise(b, u)
% Propagate the belief b forward a single step given the control inputs
% u, the process model f, and the observation model h.
%
% INPUTS:
% b: [x; P(:)] where x is nx1 vector of states and P is the vectorized 
%    nxn covariance matrix at time step k
% u: [u_1, u_2, ..., u_m]' where u is mx1 vector of control inputs
%    at time step k
%
% OUTPUTS:
% b: [x; P(:)] where x is nx1 vector of states and P is the vectorized
%    nxn covariance matrix at time step k+1
%
% Author: Jared Strader
% Structure of this code inspired by some of the code from
% https://github.com/sauravag/bsp-ilqg
%

tempb = zeros(size(b));

for i=1:size(b,2) 
    tempb(:,i) = update_belief_without_noise(obj, b(:,i), u(:,i));
end

b=tempb;

end

function b_kp1 = update_belief_without_noise(obj, b_k, u_k)

% check if control input is nan, if not set to zero
if any(isnan(u_k(1,:)))
    u_k = zeros(size(u_k));
%     error('Error! u is nan in belief dynamics');
end

%number of states
n = obj.x_dim_;

%state vector
x_k = b_k(1:n,1);

%covariance matrix
P_k = reshape(b_k(n+1:end), [n,n]);

%propagate state
x_kp1 = obj.propagate_state_without_noise(x_k, u_k); 

%proces jacobians
A = obj.get_process_jacobian(x_k,u_k);
L = obj.get_process_noise_jacobian(x_k,u_k);
Q = obj.get_process_noise_covariance(x_k,u_k);

%propagate covariance
P_kp1 = A*P_k*A' + L*Q*L';

%check if measurement is acquired
if(obj.is_measurement_acquired(x_kp1))
    %observation jacobians
    H = obj.get_measurement_jacobian(x_k);
    M = obj.get_measurement_noise_jacobian(x_k);
    R = obj.get_measurement_noise_covariance(x_k);
    
    %innovation covariance
    S = H*P_kp1*H' + M*R*M';
    
    %kalman gain
    K = (P_kp1*H')/S;
    
    %update covariance
    P_kp1 = (eye(n) - K*H)*P_kp1;
end

%vectorize updated states and covariance
b_kp1 = zeros(size(b_k));
b_kp1(1:n,1) = x_kp1;
b_kp1(n+1:end,1) = P_kp1(:);

end

