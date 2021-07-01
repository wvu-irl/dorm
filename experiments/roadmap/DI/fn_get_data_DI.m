function [data, traj] = fn_get_data_DI(bsp, x_goal, G, P0, guess_eig)
global robot_;

%% sequence of states and control inputs for trajectory
z = fn_get_path(x_goal(1:2), G);
x=[];
u=[];
for idx=1:length(z)-1
    tempi = z(idx);
    tempj = find(G(tempi).n == z(idx+1));
    tempx=G(tempi).x_seq(tempj).x;
    tempu=G(tempi).u_seq(tempj).u;
    
    for t=1:size(tempx,2)-1
        xt=tempx(:,t);
        ut=tempu(:,t);
        x = [x xt];
        u = [u ut];
        
        if(idx==length(z)-1 && t==size(tempx,2)-1)
            tempi = z(idx+1);
            xt=G(tempi).x;
            x = [x xt];
        end
    end
end

%% traj of vertices
traj=[];
for i=1:length(z)
    traj.v=[];
    traj.i=[];
end
%% process data for trajectory
P                   = P0; %exact value of covariance
nuPBound            = det(P0); %bound given bound with guessed eig
nuPBoundOptimal     = det(P0); %bound given bound with optimal eig
phiP                = det(P0); %bound given det with guessed eig
phiPOptimal         = det(P0); %bound given det with optimal eig
lambdaPBound        = max(eig(P0)); %max eig bound given bound
lambdaPBound_noProp = max(eig(P0)); %max eig bound given max eig
data(1).x                   = x(:,1);
data(1).u                   = u(:,1);
data(1).P                   = P;
data(1).detP                = det(P0);
data(1).nuP                 = nuPBound;
data(1).nuPOptimal          = nuPBoundOptimal;
data(1).phiP                = phiP;
data(1).phiPOptimal         = phiPOptimal;
data(1).lambdaP             = max(eig(P0));
data(1).lambdaPBound        = lambdaPBound;
data(1).lambdaPBound_noProp = lambdaPBound_noProp;
data(1).is_acquired         = 0;

for t=1:size(u,2)
    %% states and controls inputs at time step t
    xt=x(:,t);
    ut=u(:,t);
    xtp1=x(:,t+1);
    
    %% process matrices
    L = bsp.robot_.get_process_noise_jacobian(xt,ut);
    Q = bsp.robot_.get_process_noise_covariance(xt);
    Q_tilde = L*Q*L';
    F = bsp.robot_.get_process_jacobian(xt,ut);
    
    Pprev=[];
    max_eigenvalue=[];
    is_acquired = bsp.robot_.is_measurement_acquired(xtp1);
    if(is_acquired)
        %% measurement matrices
        M = bsp.robot_.get_measurement_noise_jacobian(xtp1);
        R = bsp.robot_.get_measurement_noise_covariance(xtp1);
        R_tilde = M*R*M';
        H = bsp.robot_.get_measurement_jacobian(xtp1);

        %% propagate covariance matrix
        Pprev = P;
        max_eigenvalue = max(eig(Pprev));
        P = inv(inv(F*P*F' + Q_tilde) + H'*inv(R_tilde)*H);
                
        %% propagate max eig bound given bound
        lambdaFF = max(eig(F*F'));
        lambdaQ = max(eig(Q_tilde));
        lambdaR = min(eig(H'*inv(R)*H));
        num = lambdaFF*lambdaPBound + lambdaQ;
        den = lambdaR*(lambdaFF*lambdaPBound + lambdaQ) + 1;
        lambdaPBound = num/den;
        
        %% propagate max eig bound given max eig
        num = lambdaFF*max_eigenvalue + lambdaQ;
        den = lambdaR*(lambdaFF*max_eigenvalue + lambdaQ) + 1;
        lambdaPBound_noProp = num/den;
        
        %% propagate det bound given bound with guessed eig
%         nu_Qgrave = det(guess_eig*eye(bsp.robot_.x_dim_) + Q_tilde);
%         iota_R = (lambdaFF*guess_eig + lambdaQ)^2*det(inv(R_tilde));
        [nu_Qgrave,iota_R] = bsp.get_nuQgrave_iotaR(xt, ut, xtp1, guess_eig);
        num = nuPBound + nu_Qgrave - guess_eig^bsp.robot_.x_dim_;
        den = iota_R*(nuPBound + nu_Qgrave - guess_eig^bsp.robot_.x_dim_) + 1;
        nuPBound = num/den;

        %% propagate det bound given bound with optimal eig
%         nu_Qgrave = det(max_eigenvalue*eye(bsp.robot_.x_dim_) + Q_tilde);
%         iota_R = (lambdaFF*max_eigenvalue + lambdaQ)^2*det(inv(R_tilde));
        [nu_Qgrave,iota_R] = bsp.get_nuQgrave_iotaR(xt, ut, xtp1, max_eigenvalue);
        num = nuPBoundOptimal + nu_Qgrave - max_eigenvalue^bsp.robot_.x_dim_;
        den = iota_R*(nuPBoundOptimal + nu_Qgrave - max_eigenvalue^bsp.robot_.x_dim_) + 1;
        nuPBoundOptimal = num/den;
        
        %% det bound given det at prev step using guessed eig
%         nu_Qgrave = det(guess_eig*eye(bsp.robot_.x_dim_) + Q_tilde);
%         iota_R = (lambdaFF*guess_eig + lambdaQ)^2*det(inv(R_tilde));
        [nu_Qgrave,iota_R] = bsp.get_nuQgrave_iotaR(xt, ut, xtp1, guess_eig);
        num = det(Pprev) + nu_Qgrave - guess_eig^bsp.robot_.x_dim_;
        den = iota_R*(det(Pprev) + nu_Qgrave - guess_eig^bsp.robot_.x_dim_) + 1;
        phiP = num/den;
        
        %% det bound given det at prev step using optimal eig
        max_eigenvalue = max(eig(data(t).P));
%         nu_Qgrave = det(max_eigenvalue*eye(bsp.robot_.x_dim_) + Q_tilde);
%         iota_R = (lambdaFF*max_eigenvalue + lambdaQ)^2*det(inv(R_tilde));
        [nu_Qgrave,iota_R] = bsp.get_nuQgrave_iotaR(xt, ut, xtp1, max_eigenvalue);
        num = det(Pprev) + nu_Qgrave - max_eigenvalue^bsp.robot_.x_dim_;
        den = iota_R*(det(Pprev) + nu_Qgrave - max_eigenvalue^bsp.robot_.x_dim_) + 1;
        phiPOptimal = num/den;
        
    else
        %% propagate covariance matrix
        Pprev = P;
        max_eigenvalue = max(eig(Pprev));
        P = F*P*F' + Q_tilde;

        %% propagate max eig bound given bound
        lambdaFF = max(eig(F*F'));
        lambdaQ = max(eig(Q_tilde));
        lambdaPBound = lambdaFF*lambdaPBound + lambdaQ;
        
        %% propagate max eig bound given max eig
        lambdaFF = max(eig(F*F'));
        lambdaQ = max(eig(Q_tilde));
        lambdaPBound_noProp = lambdaFF*max_eigenvalue + lambdaQ;
        
        %% propagate det bound given det bound using guessed eig
        nu_Qgrave = det(guess_eig*eye(bsp.robot_.x_dim_) + Q_tilde);
        nuPBound = nuPBound + nu_Qgrave - guess_eig^bsp.robot_.x_dim_;

        %% propagate det bound given bound with optimal eig
        max_eigenvalue = max(eig(data(t).P));
        nu_Qgrave = det(max_eigenvalue*eye(bsp.robot_.x_dim_) + Q_tilde);
        nuPBoundOptimal = nuPBoundOptimal + nu_Qgrave - max_eigenvalue^bsp.robot_.x_dim_;
        
        %% det bound given det at prev step using guessed eig
        nu_Qgrave = det(guess_eig*eye(bsp.robot_.x_dim_) + Q_tilde);
        phiP = det(Pprev) + nu_Qgrave - guess_eig^bsp.robot_.x_dim_;
        
        %% det bound given det at prev step using optimal eig
        max_eigenvalue = max(eig(data(t).P));
        nu_Qgrave = det(max_eigenvalue*eye(bsp.robot_.x_dim_) + Q_tilde);
        phiPOptimal = det(Pprev) + nu_Qgrave - max_eigenvalue^bsp.robot_.x_dim_;
    end
    
    %% save data
    data(t).u = ut;
    data(t+1).x = xtp1;
    data(t+1).P = P;
    data(t+1).detP = det(P);
    data(t+1).nuP = nuPBound;
    data(t+1).nuPOptimal = nuPBoundOptimal;
    data(t+1).phiP = phiP;
    data(t+1).phiPOptimal = phiPOptimal;
    data(t+1).lambdaP = max(eig(P));
    data(t+1).lambdaPBound = lambdaPBound;
    data(t+1).lambdaPBound_noProp = lambdaPBound_noProp;
    data(t+1).is_acquired = is_acquired;
    if(t==size(u,2))
        data(t+1).u = nan*zeros(size(ut));
    else
        
    end
end

end

