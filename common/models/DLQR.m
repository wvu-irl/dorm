function [u, x, h, k, J] = DLQR(x_init, A, B, N, H, Q, R)
%DLQR Solves a discritized linear quadratic regulator problem
% INPUT:
%       x_init  Initial state
%       A       State dynamics
%       B       Control Signal dynamics
%       N       Number of time steps
%       H       Final state cost
%       Q       State cost
%       R       Control cost
% OUTPUT
%       u       Control policy
%       x       System states
%       h       State cost at each timestep
%       k       Control gain at each timestep
%       J       Total cost
u = [];
x = [];
h = [];
k = [];
J = [];

H_prev = H;
K = (B'*H*B + R)\B'*H*A;
Ac = A-B*K;
H = Q + K'*R*K + Ac'*H*Ac;

if N > 0
    [u_t, x_t, h_t, k_t, J_t] = DLQR(x_init, A, B, N-1, H, Q, R);  
    len = size(x_t,2);
    u = [u_t, -K*x_t(:,len)];
    x = [x_t, A*x_t(:,len) + B*u(:,len)];
    h = [h_t, H];
    k = [k_t, K(:)];
    J = [ J_t, 0.5* x_t(:,len)'*H*x_t(:,len)];
else
    u = [];%[-K*x_init ];
    x = [x_init];%, A*x_init + B*u ];
    h = [ H ];
    k = [ K(:) ];
    J = [0,  0.5* x_init'*H*x_init ];  
end

end

