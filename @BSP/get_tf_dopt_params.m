function [xi_params] = get_tf_dopt_params(obj, x, u)
% function [xi_params] = get_tf_dopt_params(obj, x, u)
% Returns coefficients for the one-step update parameters for DORM as a 
% function of ell. Require to avoid recomputing transform function
% parameters online if the value of the maximum eigenvalue bound changes
%
% Inputs:
% x = sequence of states where x[:,i] = [x0;x1;...] is the ith state
% u = sequence of control inputs where u[:,i] = [u0;u1;...] is the ith
%     control input
%
% Outputs:
% xi_params = [pa1,pc1,pb1,pd1;...
%              pa2,pc2,pb2,pd2;...
%              pa3,pc3,pb3,pd3;...
%              pa4,pc4,pb4,pd4];
%
% The transfer function is defined as follows:
% 
% nu_{k+1} = (a*nu_k + b) / (c*nu_k + d)
%
% where
%
% a = pa1*ell^3 + pa2*ell^2 + pa3*ell + pa4
% b = pb1*ell^3 + pb2*ell^2 + pb3*ell + pb4
% c = pc1*ell^3 + pc2*ell^2 + pc3*ell + pc4
% d = pd1*ell^3 + pd2*ell^2 + pd3*ell + pd4
% 
% Authors: Jared Strader
%

ell_vec = [1,10,100,1000,10000]; %values of ell for fitting the functions
xi_vec = []; %set of parameters for each value of ell
for idx=1:length(ell_vec)
    xi = obj.get_tf_dopt(x,...
                         u,...
                         ell_vec(idx));
    xi_vec(:,idx) = xi;
end

%fit 3rd order polynomial for each of the one-step update parameters
p1 = polyfit(ell_vec,xi_vec(1,:),3);
p2 = polyfit(ell_vec,xi_vec(2,:),3);
p3 = polyfit(ell_vec,xi_vec(3,:),3);
p4 = polyfit(ell_vec,xi_vec(4,:),3);
xi_params = [p1',p2',p3',p4'];

end

