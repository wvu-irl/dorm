classdef AckermannModel2DwithGPS < GenericStateSpaceModel
    %% Properties
    properties (Constant = true)
        x_dim_ = 4; %x,y position, orientation, steering angle
        u_dim_ = 2; % speed, rate of change of steering angle
        w_dim_ = 2; %noise derivatives of speed and steering angle
        y_dim_ = 2; %x and y position
        v_dim_ = 2; %noise in x and y positions

        car_len = 1;
    end
    
    properties
        Q_; %process covariance matrix
        R_; %measurement covariance matrix
        dt_; %sampling time
        gps_regions_; %rectangles where GPS measurements can be acquired
    end
    
    methods
        %% Constructor
        function obj = Ackermann2DwithGPS(sigma_uv,...
                                         sigma_uk,...
                                         sigma_x,...
                                         sigma_y,...
                                         dt,...
                                         gps_regions)
            obj@GenericStateSpaceModel();      
            obj.Q_ = diag([sigma_uv^2, sigma_uk^2]);
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
            w = obj.get_process_noise(x, u, dt);
            x(1) = x(1) + obj.dt_*( u(1) + w(1) )*cos(x(3)); % X
            x(2) = x(2) + obj.dt_*( u(1) + w(1) )*sin(x(3)); % Y
            x(3) = x(3) + obj.dt_*tan(x(4))/obj.car_len;      % Theta
            x(4) = x(4) + obj.dt_*(u(2) + w(2));  % Psi
        end
        
        function x = propagate_state_without_noise(obj, x, u)
            x(1) = x(1) + obj.dt_*u(1)*cos(x(3)); % X
            x(2) = x(2) + obj.dt_*u(1)*sin(x(3)); % Y
            x(3) = x(3) + obj.dt_*tan(x(4))/l;      % Theta
            x(4) = x(4) + obj.dt_*u(2);  % Psi
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
            %Ad = I + A*dt
            A_continuous = obj.get_process_matrix_continuous(x,u);
            A = eye(obj.x_dim_) + A_continuous*obj.dt_; 
        end
        
        function B = get_control_jacobian(obj, x, u)
            %Bd = B*dt
            B_continuous = obj.get_control_matrix_continuous(x,u);
            B = obj.dt_*B_continuous; 
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
            A = zeros(4); 
        end
        
        function B = get_control_matrix_continuous(obj, x, u)
            B = [cos(x(3)),0;...
                 sin(x(3)),0;...
                 tan(x(4)/obj.car_len),0;...
                 0,1];
        end
        
        %% Observation Model
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%measurement%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        function z = get_measurement_with_noise(obj, x)
            z=[];
            is_in_rect = Rectangle.is_in_rect_pts(x, obj.gps_regions_);
            if(is_in_rect)
                v = obj.get_measurement_noise(x);
                z = x(1:2) + v;
            end
        end
        
        function z = get_measurement_without_noise(obj, x)
            z=[];
            is_in_rect = Rectangle.is_in_rect_pts(x, obj.gps_regions_);
            if(is_in_rect)
                z = x(1:2);
            end
        end
        
        function is_acquired = is_measurement_acquired(obj,x)
            is_acquired=0;
            is_in_rect = Rectangle.is_in_rect_pts(x, obj.gps_regions_);
            if(is_in_rect)
                is_acquired=1;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%measurement noise%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function v = get_measurement_noise(obj, x)
            R = obj.get_measurement_noise_covariance(x);
            v = [randn*sqrt(R(1,1)); randn*sqrt(R(2,2))];
        end
        
        function R = get_measurement_noise_covariance(obj,x)
            R=[];
            is_in_rect = Rectangle.is_in_rect_pts(x, obj.gps_regions_);
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
            M = eye(obj.v_dim_);
        end
        
        %% Compute states and control inputs using LQR
        % Inputs:
        % xi = [x;y], starting state
        % xf = [x;y], goal state
        % dt = time step in seconds
        function [x,u] = get_states_and_control_inputs(obj, xi, xf)
            K = obj.get_control_law_lqr(xi);
            
            %compute states and control inputs
            x=xi-xf; %moves setpoint from xf to [0,0]
            u=[];
            while 1          
                A = obj.get_process_jacobian(x(:,end),[]);
                B = obj.get_control_jacobian(x(:,end),[]);
%                 c = obj.propagate_state_without_noise(x(:,end)+xf,zeros(obj.u_dim_,1));
%                 B=[B,c];
%                 K = obj.get_control_law_lqr(x);
                u = [u, -K*(x(:,end))];
                x = [x, A*x(:,end) + B*u(:,end)];
                
%                 xtemp = obj.propagate_state_without_noise(x(:,end)+xf, u(:,end));
%                 x = [x, xtemp]

                %check for convergence
                if(norm(x(:,end) - x(:,end-1)) < 1e-3)
                    break;
                end
            end
            x = x + xf; %move setpoint to xf
        end
        
        %% Compute control law using LQR
        % Inputs:
        % dt = time step in seconds
        function [K] = get_control_law_lqr(obj,x)
            %model
            A = obj.get_process_jacobian(x,[]);
            B = obj.get_control_jacobian(x,[]);
%             c = obj.propagate_state_without_noise(x,zeros(obj.u_dim_,1));
%             B=[B,c];

            
            %weight matrices
%             W = [1,0,0,0,0;...
%                  0,1,0,0,0;...
%                  0,0,1,0,0;...
%                  0,0,0,0,0;...
%                  0,0,0,0,0];
            W = 0.1*eye(5); %weight on states
            V = 10000*eye(2); %weight on control inputs
%             V = eye(3); %weight on control inputs

            %solve discrete time algebraic ricatti equation (DARE) for
            %regulator problem, given by (Q and R are weights)
            %P_k = Q + A'*P_{k+1}*A - A'*P_{k+1}*B*(R + B'*PP_{k+1}*B)^-1*B'*P_{k+1}*A
            %where Q=Qk, R=Rk, A=Ak, B=Bk with the control law is given by
            %uk = -(R + B'*P_{k+1})^-1*B'*P_{k+1}*A*xk
            %where R=Rk, A=Ak, B=Bk
            [P,K,L] = dare(A, B, W, V, [], []);
            K = (V + B'*P*B)^-1*B'*P*A; %steady state gain
            
            %using lqr function
%             [K,S,e] = lqr(A,B,W,V);
            
        end
        
        %%
%         function [K,H] = get_control_law_dlqr(obj,x,N)
%             % Simple function for executing the dlqr algorithm for finite 
%             % N (dlqr algorithm) and infinite N (fixed point iteration).
%             if(~isinf(N)) %solve for finite N
%                 %model
%                 A = obj.get_process_jacobian(x,[]);
%                 B = obj.get_control_jacobian(x,[]);
%                 
%                 %weights
%                 Q = eye(5);
%                 R = eye(2);
%                 H = eye(2);
%                 
%                 %dlqr algorithm
%                 tempx = xf;
%                 tempK = [];
%                 tempH = [];
%                 for k=N:-1:1
%                     %update gains
%                     K = inv(R + B'*H*B)*B'*H*A;
%                     H = Q + K'*R*K + (A-B*K)'*H*(A-B*K);
%                     tempK = [K(:) tempK];
%                     tempH = [H(:) tempH];
%                 end
%                 K = tempK;
%                 H = tempH;
%             else %solve for infinite N (using fixed point iteration)
%                 eps = inf;
%                 while eps > 1e-6
%                     Hp = H;
%                     H = Q + A'*H*A - A'*H*B*inv(R+B'*H*B)*B'*H*A;
%                     eps = norm(H-Hp);
%                 end
%                 K = inv(R + B'*H*B)*B'*H*A;
%             end
%         end
        
%         function dz = dynamics(z,u)
%             %differential equation representing dynamics
%             dx = z(4,:)*cos(z(3,:));
%             dy = z(4,:)*sin(z(3,:));
%             dth = z(4,:)*z(5,:);
%             dv = u(1,:);
%             dk = u(2,:);
%             dz = [dx;dy;dth;dv;dk;dz];
%         end
        
    end
    
end