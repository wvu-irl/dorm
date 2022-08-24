classdef DoubleIntegrator2DwithGPS < GenericStateSpaceModel
    %% Properties
    properties (Constant = true)
        x_dim_ = 4; %x and y position, x and y velocity
        u_dim_ = 2; %x and y acceleration
        w_dim_ = 2; %noise in x and y accelerations
        y_dim_ = 2; %x and y position
        v_dim_ = 2; %noise in x and y positions

        
    end
    
    properties
        Q_; %process covariance matrix
        R_; %measurement covariance matrix
        dt_; %sampling time
        gps_regions_; %rectangles where GPS measurements can be acquired

        Qp_ = diag([0.1 0.1 0]);
        Rp_ = 0.01*eye(2);
        Qt_ = diag([1 5 100]); 
        Rt_ = 0.1*eye(2);
        N_ = 100;
        mpcN_ = 40;
        predict_horizon_ = 10;
        control_horizon_ = 10;

        mpcverbosity('off');
        nlobj_ = nlmpc(x_dim_,x_dim,u_dim_);
        nlobj_.MV(1).Min = 0;
        nlobj_.MV(1).Max = 1;
        nlobj_.MV(2).Min = -pi/4;
        nlobj_.MV(2).Max = pi/4;

        nlobj_.Model.StateFcn = "state_fcn";
        nlobj_.Jacobian.StateFcn = "state_jacobian_fcn";

        % nlobj_.Optimization.CustomCostFcn = "cost_fcn";
        % nlobj_.Optimization.ReplaceStandardCost = true;
        % nlobj_.Jacobian.CustomCostFcn = "cost_jacobian_fcn";

        %nlobj_.Optimization.CustomIneqConFcn = "inequality_constraints_fcn";
        %nlobj_.Jacobian.CustomIneqConFcn = "inequality_constraints_jacobian_fcn";

        nlobj_.Optimization.SolverOptions.FunctionTolerance = 0.01;
        nlobj_.Optimization.SolverOptions.StepTolerance = 0.01;
        nlobj_.Optimization.SolverOptions.ConstraintTolerance = 0.01;
        nlobj_.Optimization.SolverOptions.OptimalityTolerance = 0.01;
        nlobj_.Optimization.SolverOptions.MaxIter = mpcN;

        % opt = nlmpcmoveopt;
        % opt.X0 = [linspace(x0(1),goal(1),predict_horizon)', ...
        %           linspace(x0(2),goal(2),predict_horizon)', ...
        %           linspace(x0(3),goal(3),predict_horizon)', ...
        %           linspace(x0(4),goal(4),predict_horizon)'];
        % opt.MV0 = zeros(predict_horizon,nu);

        %paras = {goal,Qp,Rp,Qt,Rt}';
        % paras = goal;
        % nlobj.Model.NumberOfParameters = 1;%numel(paras);
        % opt.Parameters = paras;
    end
    
    methods
        %% Constructor
        function obj = DoubleIntegrator2DwithGPS(sigma_ax,...
                                                 sigma_ay,...
                                                 sigma_x,...
                                                 sigma_y,...
                                                 dt,...
                                                 gps_regions)
            obj@GenericStateSpaceModel();      
            obj.Q_ = diag([sigma_ax^2, sigma_ay^2]);
            obj.R_ = diag([sigma_x^2, sigma_y^2]);
            obj.dt_ = dt;
            obj.gps_regions_ = gps_regions;
        end
        
        %% GPS Regions 
        %See Rectangle object in commons folder for more details
        function [] = set_gps_regions(obj, gps_regions)
            obj.gps_regions_ = gps_regions;
        end
        
        function [] = add_gps_region(obj, gps_regions)
            obj.gps_regions_ = [obj.gps_regions_ gps_regions];
        end
        
        function [] = clear_gps_regions(obj)
            obj.gps_regions_ = [];
        end
        
        %% Process Model        
        %%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%process%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%
        function x = propagate_state_with_noise(obj, x, u)
            w = obj.get_process_noise(x, u);
            A = obj.get_process_jacobian([], []);
            B = obj.get_control_jacobian([], []);
            x = A*x + B*(u+w);
        end
        
        function x = propagate_state_without_noise(obj, x, u)
            A = obj.get_process_jacobian([], []);
            B = obj.get_control_jacobian([], []);
            x = A*x + B*u;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%process noise%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function w = get_process_noise(obj, x, u)
           Q = obj.get_process_noise_covariance(x, u);
           w = [randn*sqrt(Q(1,1)); randn*sqrt(Q(2,2))];
        end
        
        function Q = get_process_noise_covariance(obj, x, u)
            Q = obj.Q_;
        end
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%process jacobians%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function A = get_process_jacobian(obj, x, u)
            A = obj.get_process_matrix_continuous([], []);
            I = eye(size(A));
            A = I + A*obj.dt_; %Ad = I + A*dt
        end
        
        function B = get_control_jacobian(obj, x, u)
            B = obj.get_control_matrix_continuous([], []);
            B = B*obj.dt_; %Bd = B*dt
        end
        
        function L = get_process_noise_jacobian(obj, x, u)
            L = [0,       0;...
                 0,       0;...
                 obj.dt_, 0;...
                 0,       obj.dt_];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%continuous%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        function A = get_process_matrix_continuous(obj, x, u)
            A = [0,0,1,0;...
                 0,0,0,1;...
                 0,0,0,0;...
                 0,0,0,0]; 
        end
        
        function B = get_control_matrix_continuous(obj, x, u)
            B = [0,0;...
                 0,0;...
                 1,0;...
                 0,1]; 
        end
        
        %% Observation Model
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%measurement%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        function z = get_measurement_with_noise(obj, x)
            z=[];
            is_in_rect = Rectangle.is_in_rect_pts(x(1:2,1), obj.gps_regions_);
            if(is_in_rect)
                v = obj.get_measurement_noise(x(1:2,1));
                z = x(1:2,1) + v;
            end
        end
        
        function z = get_measurement_without_noise(obj, x)
            z=[];
            is_in_rect = Rectangle.is_in_rect_pts(x(1:2,1), obj.gps_regions_);
            if(is_in_rect)
                z = x(1:2,1);
            end
        end
        
        function is_acquired = is_measurement_acquired(obj,x)
            is_acquired=0;
            is_in_rect = Rectangle.is_in_rect_pts(x(1:2,1), obj.gps_regions_);
            if(is_in_rect)
                is_acquired=1;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%measurement noise%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function v = get_measurement_noise(obj, x)
            R = obj.get_measurement_noise_covariance(x(1:2,1));
            v = [randn*sqrt(R(1,1)); randn*sqrt(R(2,2))];
        end
        
        function R = get_measurement_noise_covariance(obj,x)
            R=[];
            is_in_rect = Rectangle.is_in_rect_pts(x(1:2,1), obj.gps_regions_);
            if(is_in_rect)
                R = obj.R_;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%measurement jacobians%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function H = get_measurement_jacobian(obj, x)
            H = [1,0,0,0;...
                 0,1,0,0]; 
        end
        
        function M = get_measurement_noise_jacobian(obj, x)
            M = [1,0;...
                 0,1];
        end
        
        %% Compute states and control inputs using LQR
        % Inputs:
        % xi = [x;y], starting state
        % xf = [x;y], goal state
        % dt = time step in seconds
        function [x, u, K] = get_states_and_control_inputs(obj, x0, xf)
            K = 0;
            xi = x0;
            x = [xi];
            ui = [0,0];
            u = [ui'];
            while 1
                ui = nlmpcmove(nlobj_,xi,ui,xf);
                u = [u, ui'];
                xi = xi + dt*[cos(x(3)),   0;
                              sin(x(3)),   0;
                              tan(x(4))/l, 0;
                              0,           1]*ui;
                x = [x, xi];
                
                %check for convergence
                if(norm(x(:,end)) < 1e-2)
                    break;
                end
            end
        end
        
    end
    
end