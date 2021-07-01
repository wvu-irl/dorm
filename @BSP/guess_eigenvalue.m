function [eigenvalue] = guess_eigenvalue(obj,...
                                         G,...
                                         idx_start,...
                                         idx_goal,...
                                         P0)
    %% Dijkstras
    cost = inf*ones(length(G),1);
    pred = -1*ones(length(G),1);
    q=1:length(G);
    cost(idx_start)=0;
    while(~isempty(q))
        min_cost = inf;
        min_idx = -1;
        for i=1:length(q)
            tempi = q(i);
            if(cost(tempi) < min_cost)
                min_cost = cost(tempi);
                min_idx = i;
            end
        end
        if(min_idx==-1)
            break;
        else
            u = q(min_idx);
            q(min_idx)=[];
        end
        
        for v=G(u).n
            temp = cost(u) + norm(G(u).x(1:2) - G(v).x(1:2));
            if(temp < cost(v))
                cost(v) = temp;
                pred(v) = u;
            end
        end
    end
    
    %% Shortest Path
    p = idx_goal;
    u = p;
    while 1
        if(u == idx_start)
            break;
        end
        u = pred(u);
        p = [u p];
    end
    
    %% Maximum Eigenvalue
    P=P0;
    eigenvalue = max(eig(P));
    [x,u] = obj.get_path_given_indices(G,p);
    for t=1:size(x,2)-1
        %states and controls inputs at time step t
        xt=x(:,t);
        ut=u(:,t);
        xtp1=x(:,t+1);
        
        %process matrices
        L = obj.robot_.get_process_noise_jacobian(xt,ut);
        Q = obj.robot_.get_process_noise_covariance(xt);
        Q_tilde = L*Q*L';
        F = obj.robot_.get_process_jacobian(xt,ut);
        P = F*P*F' + Q_tilde;
        
%         is_acquired = robot_.is_measurement_acquired(xtp1);
%         if(is_acquired)
%             %measurement matrices
%             M = obj.robot_.get_measurement_noise_jacobian(xtp1);
%             R = obj.robot_.get_measurement_noise_covariance(xtp1);
%             R_tilde = M*R*M';
%             H = obj.robot_.get_measurement_jacobian(xtp1);
% 
%             %propagate covariance matrix
%             P = inv(inv(P) + H'*inv(R_tilde)*H);
%         end
        
        ell = max(eig(P));
        if(eigenvalue < ell)
            eigenvalue = ell;
        end
    end
end

