function [xi, GammaProd] = get_tf_dopt(obj, x, u, ell)
% function [xi, GammaProd] = get_tf_dopt(obj, x, u, ell)
% Return transfer function parameters for propagating the upper bound
% of the D-optimality criteria in a single step given a sequence of
% states and control inputs
%
% Inputs:
% x = sequence of states e.g., x[:,i] = [x;y]
% u = sequence of control inputs e.g., u[:,i] = [vx, vy]
% dt = time step in seconds
%
% Author: Jared Strader

GammaProd=eye(2);
xi = [1,0,0,1]';
for i=1:size(x,2)-1
    xt=x(:,i);
    ut=u(:,i);
    xtp1=x(:,i+1);

    gamma0=[];
    gamma1=[];

    is_acquired=obj.robot_.is_measurement_acquired(xtp1);
    if(is_acquired)
        [nu_Qgrave,iota_R] = obj.get_nuQgrave_iotaR(xt,...
                                                    ut,...
                                                    xtp1,...
                                                    ell);
        gamma0 = nu_Qgrave - ell^obj.robot_.x_dim_; 
        gamma1 = iota_R;
    else
        [nu_Qgrave] = obj.get_nuQgrave(xt,...
                                       ut,...
                                       ell);
        gamma0 = nu_Qgrave - ell^obj.robot_.x_dim_; 
        gamma1 = 0;
    end

    GammaVal = [1, gamma0;gamma1, gamma0*gamma1 + 1];
    GammaProd = GammaVal*GammaProd;
    
    xi = [GammaProd(1,1); GammaProd(1,2); GammaProd(2,1); GammaProd(2,2)];
end


end

