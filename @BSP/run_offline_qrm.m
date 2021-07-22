function [] = run_offline_qrm(obj, iterations)
% function [] = run_offline(obj, iterations)
%
% Computes D-Optimality Roadmap (DORM), which can be queried online for 
% planning a path of minimum uncertainty from a start state to a goal 
% state given an initial belief.
%
% Inputs:
% iterations = number of samples to generate the roadmap where the number
%              vertices may be less than the number of iterations as the
%              samples drawn inside obstacles are not resampled while
%              constructing the roadmap.
%
% Authors: Jared Strader

disp('Constructing DORM (using Hammersley Sampling)...');

%% 1. Sample means
disp('sampling means...');
obj.G_=[];
iter=0;
k=1;
while iter<iterations
    %dimension of configuration space
    n_configurations= size(obj.config_limits_,1); 
    if(n_configurations~=2)
        error('run_offline_qrm only implemented for 2D configuration space!');
    end
    
    %dimension of velocities
    n_velocities = size(obj.vel_limits_,1); 
    
    %used to determine if sampling velocities as well as configurations
    N=[];
    if(isempty(obj.vel_limits_))
        N=n_configurations;
    else
        N=n_configurations+n_velocities;
    end
    
    %hammersley sampling
    if(isprime(k))
        iter=iter+1;
    else
        k=k+1;
        continue;
    end
    
    %sample states
    x_rand=zeros(obj.robot_.x_dim_, 1);
    for i=1:N
        if(i<=n_configurations)
            %sample configurations
            if(i==1)
                x_rand(i,1) = obj.config_limits_(i,1) + iter/iterations*(obj.config_limits_(i,2) - obj.config_limits_(i,1));
            elseif(i==2)
                x_rand(i,1) = obj.config_limits_(i,1) + iter/iterations*(obj.config_limits_(i,2) - obj.config_limits_(i,1));
                temp = de2bi(iter);
                x_rand(i,1) = 0;
                for j=1:length(temp)
                    x_rand(i,1) = x_rand(i,1) + temp(j)/2^j;
                end
                x_rand(i,1) = obj.config_limits_(i,1) + x_rand(i,1)*(obj.config_limits_(i,2) - obj.config_limits_(i,1));
            else
                error('Number of configuration is greater than 2! run_offline_qrm only implemented for 2D.');
            end
        elseif(i<=n_velocities)
            %sample velocities
            x_rand(i,1) = obj.vel_limits_(i,1) + rand*(obj.vel_limits_(i,2) - obj.vel_limits_(i,1));
        else
            error('Attempting to sample states that are not velocities or configurations!');
        end 
    end
    
    %check if in free space
    if(Rectangle.is_in_rect_pts(x_rand(1:2),obj.obstacles_))
        continue;
    end
    
    %add vertex to graph
    v_new.x = x_rand; %state
    v_new.n = []; %neighbors
    v_new.i = length(obj.G_) + 1; %index
    v_new.xi = []; %tf parameters for dopt roadmap
    v_new.xi_coeff = []; %coefficients for tf parameters dopt roadmap
    v_new.tf_brm = []; %transfer functions for covariance matrix
    v_new.tf_rbrm = []; %transfer functions for covariance matrix
    v_new.x_seq = []; %sequence of states for incident edges
    v_new.u_seq = []; %sequence of control inputs for incident edges
    obj.G_ = [obj.G_ v_new];
end

%% 2. Construct edge set
disp('constructing edge set...');
%connect vertices
for i=1:length(obj.G_)    
    %points within ball radius of kth sampled point
    U = fn_near(obj.G_,...
                obj.G_(i).x,...
                obj.radius_);
    
    %connect sampled point to neighborhood
    for j=1:length(U) 
        if(U(j).i~=i)
            traj = fn_quick_steer(U(j).x(1:2),...
                                  obj.G_(i).x(1:2),...
                                  obj.radius_); %quick steer connects with straight line instead of using dynamics, TODO: add option to use dynamics
            if(~Rectangle.is_in_rect_line(traj(:,1),traj(:,end),obj.obstacles_)) %TODO: if not used quick steer, use appropriate function
                obj.G_(i).n        = [obj.G_(i).n, U(j).i]; 
                obj.G_(i).xi       = [obj.G_(i).xi, nan*ones(4,1)];
                obj.G_(i).xi_coeff = [obj.G_(i).xi_coeff, nan*ones(16,1)];
            end
        end
    end
end

% %delete vertices if not connected
% idx_delete=[];
% for i=1:length(G)
%     if(isempty(G(i).n))
%         idx_delete = [idx_delete, i];
%     end
% end
% G(idx_delete) = [];

%% 3. Compute parameters for transfer functions
for i=1:length(obj.G_)
    disp(['generating TFs for vertex ', num2str(i), ' of ', num2str(length(obj.G_))]);
    for j=1:length(obj.G_(i).n)
        %states and control inputs for edge ij
        tempj = obj.G_(i).n(j);
        [x,u,K]=obj.robot_.get_states_and_control_inputs(obj.G_(i).x,...
                                                         obj.G_(tempj).x);
        obj.G_(i).x_seq(j).x = x;
        obj.G_(i).u_seq(j).u = u;
        
        %TF for DORM
        %compute xi using recurrence relation
        xi = obj.get_tf_dopt(x,u,0);
        obj.G_(i).xi(:,j) = xi;
        
        %compute xi as a function of alpha
        xi_coeff = obj.get_tf_dopt_params(x,u);
        obj.G_(i).xi_coeff(:,j) = xi_coeff(:);
        
        %TF for BRMS
        tf_rbrm = obj.get_tf_rbrm(x,u);
        obj.G_(i).tf_rbrm(:,j) = tf_rbrm;
        
        %TF for BRM
        tf_brm = obj.get_tf_brm(x,u);
        obj.G_(i).tf_brm(:,j) = tf_brm;
    end
end

end

