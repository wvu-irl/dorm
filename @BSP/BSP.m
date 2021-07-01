classdef BSP < handle
    properties
        %graph generated in offline phase
        G_;
        
        %radius for connecting vertices. the radius used for connecting 
        %vertices. The edges are removed based on the state generated from 
        %the control inputs; however, edges are added based on the 
        %Euclidean distance between the sampled congifurations. The 
        %velocity values are ignored for connecting sampled vertices
        radius_;
        
        %robot model (e.g., process and observation models defined based
        %on GenericStateSpaceModel, see SingleIntegrator2DwithGPS and
        %DoubleIntegrator2DwithGPS for examples)
        robot_;
        
        %limits for the robots configuration space, defined as follows:
        % config_limits_ = [x0_min, x0_max;...
        %                   x0_min, x1_max;...
        %                    .
        %                    .
        %                    .
        %                   xn_min, xn_max];
        %
        % For example, config_limits_(1,:) might be the limits of the 
        % x position and config_limits_(2,:) might be the limits of the y 
        % position. However, note these are not limited to this example.
        %
        % Important, the state vector is assumed to be ordered as follows:
        % X = [x0, x1, ... v0, v1, ...]; where x represets configuration
        % variables and v represents velocity variables
        config_limits_=[];
        
        %limits for the robots velocity space (i.e., derivatives of the
        %configurations)
        % vel_limits_ = [v0_min, v0_max;...
        %                v0_min, v1_max;...
        %                  .
        %                  .
        %                  .
        %                vn_min, vn_max];
        %
        % For example, vel_limits_(1,:) might be the limits of the x 
        % velocity and vel_limits_(2,:) might be the limits of the y 
        % velocity. However, note these are not limited to this example.
        %
        % Important, the state vector is assumed to be ordered as follows:
        % X = [x0, x1, ... v0, v1, ...]; where x represets configuration
        % variables and v represents velocity variables
        vel_limits_=[];
        
        %obstacles represented as rectangles (i.e., see Rectangle class
        %for details on defining obstacles)
        obstacles_=[];
    end
    
	%% Public
    methods (Access = public)
        function obj = BSP(robot,...
                           config_limits,...
                           vel_limits,...
                           obstacles,...
                           radius)   
            obj.robot_         = robot;
            obj.config_limits_ = config_limits;
            obj.vel_limits_    = vel_limits;
            obj.obstacles_     = obstacles;
            obj.radius_        = radius;
        end
        
        %runs offline phase
        [] = run_offline(obj,...
                         iterations,...
                         radius);
                     
        %runs online phase (returns maximum eigenvalue approximated and 
        %used in online phase)
        [eigenvalue] = run_online(obj,...
                                  x0,...
                                  P0,...
                                  x_goal,...
                                  epsilon,...
                                  eigenvalue,...
                                  method);
                              
        %fns for computing parameters for determinant bound
        [nu_Qgrave]        = get_nuQgrave(obj, x, u, ell);
        [nu_Qgrave,iota_R] = get_nuQgrave_iotaR(obj, xprev, u, xnext, ell);
                              
        %fns for computing transfer functions (i.e., one-step updates)
        tf_dorm       = get_tf_dopt(obj,x,u,ell);
        tf_dorm_coeff = get_tf_dopt_params(obj,x,u);
        tf_rbrm       = get_tf_rbrm(obj,x,u);
        tf_brm        = get_tf_brm(obj,x,u);
    end
        
    %% Protected
    methods (Access = protected)
        %fns for constructing graph
        [X,indices]    = near(obj, G, x, nu);
        [x_nearest]    = nearest(obj, x, G);
        [z]            = quick_steer(obj, x, y, nu);
        [path_indices] = get_path_given_goal(obj, G, x_goal);
        [x,u]          = get_path_given_indices(obj, G, z);
        
        %fns for online phase
        [eigenvalue] = guess_eigenvalue(obj,G,idx_start,idx_goal,P0);
        [n]          = find_min_cost(obj, q, G);
    end
    
end