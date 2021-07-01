function [eigenvalue] = run_online(obj,...
                                   x0,...
                                   P0,...
                                   x_goal,...
                                   epsilon,...
                                   eigenvalue,...
                                   method)
% function [eigenvalue] = run_online(obj,...
%                                    x0,...
%                                    P0,...
%                                    x_goal,...
%                                    epsilon,...
%                                    eigenvalue,...
%                                    method)
%
% Searches roadm for path using selected method from start state x0 to
% goal state x_goal given the initial belief defined by (x0,P0) using the
% parameters epsilon, eigenvalue, method
%
% Inputs:
% x0 = mean of initial belief
% P0 = covariance matrix of initial belief
% epsilon = parameter for pruning paths using partial ordering, only used
%           for DORM
% eigenvalue = guess for maximum eigenvalue, set to -1 for auto guess
% method = parameter for choosing method, 1=DORM, 2=BRMS, or 3=BRM
%
% Outputs:
% eigenvalue = return the eigenvalue used for DORM (used to check
%              eigenvalue selected from auto guess
%
% Authors: Jared Strader

%% Augment Vertex Set
disp('Augmenting graph...');
for i=1:length(obj.G_)
    obj.G_(i).visited = i;   %visited vertices, needed for avoiding loops
    obj.G_(i).c = inf;       %cost
    obj.G_(i).P = [];        %covariance matrix
    obj.G_(i).d = inf;       %distance, needed for partial ordering
    obj.G_(i).parent = [];   %needed to get path from start vertex
    obj.G_(i).children = []; %needed for recursive propagate
end

%% Add Start Vertex
disp('Inserting start vertex...');
%check if start state is in free space
if(Rectangle.is_in_rect_pts(x0,obj.obstacles_))
    error('Error! x0 is in obstacle!');
end

%add start vertex
v_start.x = x0;
v_start.n = [];
v_start.i = length(obj.G_) + 1;
v_start.xi = [];
v_start.xi_coeff = [];
v_start.tf_brm = [];
v_start.tf_rbrm = [];
v_start.x_seq = [];
v_start.u_seq = []; 
v_start.visited = i;
v_start.c = det(P0);
v_start.P = P0;
v_start.d = 0;
v_start.parent = [];
v_start.children = [];
obj.G_ = [obj.G_ v_start];

%points within ball radius of start vertex
U = obj.near(obj.G_,...
             obj.G_(v_start.i).x,...
             obj.radius_);

%connect start state to neighborhood
for i=1:length(U) 
    tempxstart = obj.G_(v_start.i).x(1:2); %TODO: change to generalize
    tempxgoal = U(i).x(1:2); %TODO: change to generalize
    if(~Rectangle.is_in_rect_line(tempxstart,tempxgoal,obj.obstacles_))
        %add edge v_start to v_i
        obj.G_(v_start.i).n = [obj.G_(v_start.i).n, U(i).i]; 

        %states and control inputs for edge ij
        [x,u,K]=obj.robot_.get_states_and_control_inputs(obj.G_(v_start.i).x,...
                                                         obj.G_(U(i).i).x);
        tempj = length(obj.G_(v_start.i).x_seq) + 1;
        obj.G_(v_start.i).x_seq(tempj).x = x;
        obj.G_(v_start.i).u_seq(tempj).u = u;

        %child
        tempchild = find(obj.G_(v_start.i).n == U(i).i);
        
        %compute xi using recurrence relation
        xi = obj.get_tf_dopt(x,u,0);
        obj.G_(v_start.i).xi(:,tempchild) = xi;
        
        xi_coeff = obj.get_tf_dopt_params(x,u);
        obj.G_(v_start.i).xi_coeff(:,tempchild) = xi_coeff(:);
        
        %compute tf rbrm
        xi = obj.get_tf_rbrm(x,u);
        obj.G_(v_start.i).tf_rbrm(:,tempchild) = xi;
        
        %compute tf brm
        xi = obj.get_tf_brm(x,u);
        obj.G_(v_start.i).tf_brm(:,tempchild) = xi;
    end
end

%% Add Goal Vertex
disp('Inserting goal vertex...');
%check if goal state is in free space
if(Rectangle.is_in_rect_pts(x_goal,obj.obstacles_))
    error('Error! x_goal is in obstacle!');
end

%add goal vertex
v_goal.x = x_goal;
v_goal.n = [];
v_goal.i = length(obj.G_) + 1;
v_goal.xi = [];
v_goal.xi_coeff = [];
v_goal.tf_brm = [];
v_goal.tf_rbrm = [];
v_goal.x_seq = [];
v_goal.u_seq = []; 
v_goal.visited = i;
v_goal.c = inf;
v_goal.P = [];
v_goal.d = inf;
v_goal.parent = [];
v_goal.children = [];
obj.G_ = [obj.G_ v_goal];

%points within ball radius of start vertex
U = obj.near(obj.G_,...
             obj.G_(v_goal.i).x,...
             obj.radius_);

%connect neighborhood to goal state
for i=1:length(U) 
    tempxstart = obj.G_(U(i).i).x(1:2); %TODO: change to generalize
    tempxgoal = obj.G_(v_goal.i).x(1:2); %TODO: change to generalize
    if(~Rectangle.is_in_rect_line(tempxstart,tempxgoal,obj.obstacles_))
        %add edge v_i to v_goal
        obj.G_(U(i).i).n = [obj.G_(U(i).i).n, v_goal.i];

        %states and control inputs for edge ij
        [x,u,K]=obj.robot_.get_states_and_control_inputs(obj.G_(U(i).i).x,...
                                                         obj.G_(v_goal.i).x);
        tempj = length(obj.G_(U(i).i).x_seq) + 1;
        obj.G_(U(i).i).x_seq(tempj).x = x;
        obj.G_(U(i).i).u_seq(tempj).u = u;

        %child
        tempchild = find(obj.G_(U(i).i).n == v_goal.i);
        
        %compute xi using recurrence relation
        xi = obj.get_tf_dopt(x,u,0);
        obj.G_(U(i).i).xi(:,tempchild) = xi;
        
        xi_coeff = obj.get_tf_dopt_params(x,u);
        obj.G_(U(i).i).xi_coeff(:,tempchild) = xi_coeff(:);
        
        %compute tf rbrm
        xi = obj.get_tf_rbrm(x,u);
        obj.G_(U(i).i).tf_rbrm(:,tempchild) = xi;
        
        %compute tf brm
        xi = obj.get_tf_brm(x,u);
        obj.G_(U(i).i).tf_brm(:,tempchild) = xi;
    end
end

%% Guess for Maximum Eigenvalue (if no guess provided, use shortest path)
if(eigenvalue<0)
    disp('Guessing maximum eigenvalue...');
    eigenvalue = obj.guess_eigenvalue(obj.G_,...
                                      v_start.i,...
                                      v_goal.i,...
                                      P0);
end

%% Run Belief Roadmap Search for Selected Method
if(method==1)
    disp('Running belief roadmap search (using d-opt bound)...');
elseif(method==2)
    disp('Running belief roadmap search (using e-opt bound)...');
elseif(method==3)
    disp('Running belief roadmap search (using trace of covariance)...');
else
    error('Must select valid method: 1=DORM (d-opt bound), 2=BRMS (e-opt bound), 3=BRM (trace of covariance matrix)');
end

tic
q = v_start.i; %priority queue, initialize to index of start vertex
while ~isempty(q)
    %pop
    q_idx = obj.find_min_cost(q,obj.G_);
    i=q(q_idx);
    q(q_idx) = [];
    
    %propagate for each edge if neighbors not already in path
    for j=obj.G_(i).n
        %check if j is in path of i, if so skip (not needed if cost is
        %strictly monotonically increasing in time, but this is not the
        %case for costs based on the covariance matrix such as d-opt and
        %e-opt bounds as well as exact values of the trace, maximum
        %eigenvalue, and determinant of the covariance matrix)
        skip=false;
        for k=obj.G_(i).visited
            if(j==k)
                skip=true;
                break;
            end
        end
        if(skip==true)
            continue;
        end
        
        %cost (using selected method)
        c_ij=[];
        P_ij=[];
        if(method==1) %DORM, d-opt bound, uses the guessed eigenvalue
            tempchild = find(obj.G_(i).n == j);
            xi_coeff = reshape(obj.G_(i).xi_coeff(:,tempchild),[4,4]);
            xi = [xi_coeff(1,1)*eigenvalue^3 + xi_coeff(2,1)*eigenvalue^2 + xi_coeff(3,1)*eigenvalue + xi_coeff(4,1);...
                  xi_coeff(1,2)*eigenvalue^3 + xi_coeff(2,2)*eigenvalue^2 + xi_coeff(3,2)*eigenvalue + xi_coeff(4,2);...
                  xi_coeff(1,3)*eigenvalue^3 + xi_coeff(2,3)*eigenvalue^2 + xi_coeff(3,3)*eigenvalue + xi_coeff(4,3);...
                  xi_coeff(1,4)*eigenvalue^3 + xi_coeff(2,4)*eigenvalue^2 + xi_coeff(3,4)*eigenvalue + xi_coeff(4,4)];
            tempnu = obj.G_(i).c;
            c_ij = (xi(1)*tempnu + xi(2))/(xi(3)*tempnu + xi(4));
        elseif(method==2) %BRMS, e-opt bound
            tempchild = find(obj.G_(i).n == j);
            xi = obj.G_(i).tf_rbrm(:,tempchild);
            templambda = obj.G_(i).c;
            c_ij = (xi(1)*templambda + xi(2))/(xi(3)*templambda + xi(4));
        elseif(method==3) %BRM, trace of covariance matrix
            tempchild = find(obj.G_(i).n == j);
            xi = reshape(obj.G_(i).tf_brm(:,tempchild), [2*obj.robot_.x_dim_, 2*obj.robot_.x_dim_]);
            tempPsi_prev = [obj.G_(i).P; eye(obj.robot_.x_dim_)];
            tempPsi_curr = xi*tempPsi_prev;
            Psi11 = tempPsi_curr(1:obj.robot_.x_dim_,1:end);
            Psi21 = tempPsi_curr((obj.robot_.x_dim_+1):end,1:end);
            P_ij = Psi11*inv(Psi21);
            c_ij = trace(P_ij);
        else
            error('Must select valid method: 1=DORM (d-opt bound), 2=BRMS (e-opt bound), 3=BRM (trace of covariance matrix)');
        end
        
        %compute distance (needed for partial ordering)
        d_ij = obj.G_(i).d + norm(obj.G_(j).x(1:2) - obj.G_(i).x(1:2));
        
        %See eq. 9 in BRMS paper [1] for partial odering. The partial 
        %ordering is also used in RRBT paper [2]
        %TODO: add option to disable partial ordering
        %[1] Shan, T., & Englot, B. (2017). Belief roadmap search: 
        %    Advances in optimal and efficient planning under 
        %    uncertainty. IEEE International Conference on Intelligent 
        %    Robots and Systems, 5318–5325.
        %[2] Bry, A., & Roy, N. (2011). Rapidly-exploring Random Belief 
        %    Trees for Motion Planning Under Uncertainty. IEEE 
        %    International Conference on Robotics and Automation, 723–730.
        if(c_ij < obj.G_(j).c + epsilon && d_ij < obj.G_(j).d)
%         if(c_ij < obj.G_(j).c)
            %%%%%update cost%%%%%
            %propagate bound
            obj.G_(j).c = c_ij;
            obj.G_(j).d = d_ij;
            obj.G_(j).P = P_ij; %note, this will be empty of d-opt or e-opt bounds are used
            
            %remove j from parent's children list
            if(~isempty(obj.G_(j).parent))
                p = obj.G_(j).parent;
                for k=1:length(obj.G_(p).children)
                    if(obj.G_(p).children(k)==j)
                        obj.G_(p).children(k)=[];
                        break;
                    end
                end
            end
            
            %update parent of j, children of i, and visited of j
            obj.G_(j).parent = i;
            obj.G_(i).children = [obj.G_(i).children j];
            obj.G_(j).visited = [obj.G_(i).visited j];
            
            %%%%%recursive propagate%%%%%
            %required to update costs of children after modifying a parent
            %this is necessary since using a best-first search
            tempq=obj.G_(j).children;
            while ~isempty(tempq)
                %pop
                tempi = tempq(1);
                tempq(1)=[];
                tempp = obj.G_(tempi).parent;
                
                %cost (using selected method)
                if(method==1)
                    tempchild = find(obj.G_(tempp).n == tempi);
                    xi_coeff = reshape(obj.G_(tempp).xi_coeff(:,tempchild),[4,4]);
                    xi = [xi_coeff(1,1)*eigenvalue^3 + xi_coeff(2,1)*eigenvalue^2 + xi_coeff(3,1)*eigenvalue + xi_coeff(4,1);...
                          xi_coeff(1,2)*eigenvalue^3 + xi_coeff(2,2)*eigenvalue^2 + xi_coeff(3,2)*eigenvalue + xi_coeff(4,2);...
                          xi_coeff(1,3)*eigenvalue^3 + xi_coeff(2,3)*eigenvalue^2 + xi_coeff(3,3)*eigenvalue + xi_coeff(4,3);...
                          xi_coeff(1,4)*eigenvalue^3 + xi_coeff(2,4)*eigenvalue^2 + xi_coeff(3,4)*eigenvalue + xi_coeff(4,4)];
                    tempnu = obj.G_(tempp).c;
                    obj.G_(tempi).c = (xi(1)*tempnu + xi(2))/(xi(3)*tempnu + xi(4));
                elseif(method==2)
                    tempchild = find(obj.G_(tempp).n == tempi);
                    xi = obj.G_(tempp).tf_rbrm(:,tempchild);
                    templambda = obj.G_(tempp).c;
                    obj.G_(tempi).c = (xi(1)*templambda + xi(2))/(xi(3)*templambda + xi(4));
                elseif(method==3)
                    tempchild = find(obj.G_(tempp).n == tempi);
                    xi = reshape(obj.G_(tempp).tf_brm(:,tempchild), [2*obj.robot_.x_dim_, 2*obj.robot_.x_dim_]);
                    tempPsi_prev = [obj.G_(tempp).P; eye(obj.robot_.x_dim_)];
                    tempPsi_curr = xi*tempPsi_prev;
                    Psi11 = tempPsi_curr(1:obj.robot_.x_dim_,1:end);
                    Psi21 = tempPsi_curr((obj.robot_.x_dim_+1):end,1:end);
                    tempP = Psi11*inv(Psi21);
                    obj.G_(tempi).c = trace(tempP);
                    obj.G_(tempi).P = tempP;
                else
                    error('Must select valid method: 1=DORM (d-opt bound), 2=BRMS (e-opt bound), 3=BRM (trace of covariance matrix)');
                end
                
                %compute distance
                obj.G_(tempi).d = obj.G_(tempp).d + norm(obj.G_(tempi).x(1:2) - obj.G_(tempp).x(1:2));
                
                %add children to queue
                tempq = [tempq obj.G_(tempi).children];
            end
            
            %%%%%push%%%%%
            q = [q j];
        end
    end
end
toc

end

