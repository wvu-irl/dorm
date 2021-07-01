classdef GenericStateSpaceModel < handle
    properties (Abstract, Constant)
        %% process model dimensions
        x_dim_; %length state vector
        u_dim_; %length control input vector
        w_dim_; %length process noise vector
        
        %% observation model dimensions
        y_dim_; %length output vector
        v_dim_; %length observation noise vector
    end
    
    methods (Abstract)
        %% process model (discrete)
        %propagate states
        x = propagate_state_with_noise(obj, x, u, dt)
        x = propagate_state_without_noise(obj, x, u, dt)
        
        %process noise
        w = get_process_noise(obj, x, u, dt);
        Q = get_process_noise_covariance(obj, x, u, dt)
        
        %process jacobians
        A = get_process_jacobian(obj, x, u, dt)
        B = get_control_jacobian(obj, x, u, dt)
        L = get_process_noise_jacobian(obj, x, u, dt)
        
        %% process model (continuous)
        A = get_process_matrix_continuous(obj, x, u);
        B = get_control_matrix_continuous(obj, x, u);
        
        %% observation model
        %measurement
        is_acquired = is_measurement_acquired(obj,x);
        z = get_measurement_with_noise(obj, x);
        z = get_measurement_without_noise(obj, x);
        
        %measurement noise
        v = get_measurement_noise(obj, x);
        R = get_measurement_noise_covariance(obj,x);
        
        %measurement jacobians
        H = get_measurement_jacobian(obj, x);
        M = get_measurement_noise_jacobian(obj, x);
        
        %% dynamics
%         [x,u] = compute_trajectory(obj, x_start, x_goal);
    end
    
    methods
        %% belief dynamics (using first-order approximation)
        [b, b_truth] = propagate_belief_with_noise(obj, b, u, b_truth);
        b = propagate_belief_without_noise(obj, b, u);
    end
    
end