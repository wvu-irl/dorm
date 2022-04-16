function [u, x, h, k, J] = IH_DLQR(x_init, A, B, N, H, Q, R, epsilon, alpha)
%IH_DLQR Solves a discritized linear quadratic regulator problem
%           for an infinite horizon
% INPUT:
%       x_init  Initial state
%       A       State dynamics
%       B       Control Signal dynamics
%       N       Number of time steps to output (Note, still plans
%                   for infinite horizon
%       H       Final state cost
%       Q       State cost
%       R       Control cost
%       epsilon Convergence criteria
%       alpha   Learning rate
% OUTPUT
%       u       Control policy
%       x       System states
%       h       State cost at each timestep
%       k       Control gain at each timestep
%       J       Total cost
diff = 2*epsilon;
while diff > epsilon
    H_temp = Q + A'*H*A - A'*H*B*((B'*H*B + R)\B')*H*A;
    diff = sum(sum(abs(H - H_temp)));
    H = (1-alpha)*H + (alpha)*H_temp; 
end

K = (B'*H*B + R)\B'*H*A;

u = [];
x = [x_init];
h = [H];
k = [K];
J = [];

for i = 0:N-1
    u = [u, -K*x(:,i+1)];
    x = [x, A*x(:,i+1) + B*u(:,i+1)];
    h = [h, H];
    k = [k, K];
    J = [ J, 0.5* x(:,i+1)'*H*x(:,i+1)];
end

end

