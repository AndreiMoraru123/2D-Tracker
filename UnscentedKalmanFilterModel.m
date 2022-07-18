classdef UnscentedKalmanFilterModel < handle
    
    properties
        H
        Q
        R
        state
        covariance
        innovation
        innovation_covariance
        sigma_points
        weights
        augmented_state
        augmented_covariance
    end
    
    methods
        
        function [obj] = UnscentedKalmanFilterModel()
            obj.state = [];
            obj.covariance = zeros(4,4);
            obj.innovation = [];
            obj.innovation_covariance = [];
            obj.sigma_points = [];
            obj.weights = [];
            obj.augmented_state = [];
            obj.augmented_covariance = [];
        end
        
        function [obj] = sigma_generation(obj)
            number_of_states = size(obj.augmented_state,2);
            kappa = 3 - number_of_states;
            root_cov = sqrtm(obj.augmented_covariance);
            sigma0 = obj.augmented_state;
            
            for i = 1 : number_of_states
                obj.sigma_points{i} = obj.augmented_state + sqrt(kappa + number_of_states) * root_cov(i,:);
                obj.sigma_points{number_of_states + i} = obj.augmented_state - sqrt(kappa + number_of_states) * root_cov(i,:);
            end
                      
            sigma{1} = sigma0;
            
            for i = 2 : 2 * number_of_states + 1
                sigma{i} =  obj.sigma_points{i-1};
            end
            
            obj.sigma_points = sigma;
        end
        
        function [obj] = weight_generation(obj)
            number_of_states = size(obj.augmented_state,2);
            kappa = 3 - number_of_states;
            w0 = kappa / (kappa + number_of_states);
            wi = 0.5 / (kappa + number_of_states);
            obj.weights(1) = w0;
            
            for i = 2 : 2 * number_of_states + 1
                obj.weights(i)  = wi;
            end
        end
        
        function [obj] = initialise(obj, accel_std, yaw_std, meas_std, init_on_measurement, init_pos_std, init_vel_std, init_yaw_std, measurement, varargin)
            obj.H = [1 0 0 0;
                0 1 0 0];
            obj.Q = diag([0.001, 0.001, yaw_std*yaw_std, accel_std*accel_std]);
            obj.R = diag([meas_std*meas_std,meas_std*meas_std]);
            
            if init_on_measurement == false
                obj.state = [0 0 0 0];
                obj.covariance = diag([init_pos_std*init_pos_std,init_pos_std*init_pos_std,init_yaw_std*init_yaw_std,init_vel_std*init_vel_std]);
            else
                obj.state = [measurement(1) measurement(2) 0 0];
                obj.covariance = diag([init_pos_std*init_pos_std, init_pos_std*init_pos_std, init_yaw_std*init_yaw_std, init_vel_std*init_vel_std]);
            end
        end
        
        function [obj] = prediction_step(obj, time_step, yaw_rate)
            dt = time_step;
            
            if ~isempty(obj.state)
                x = obj.state;
                
                P = obj.covariance;
                
                number_of_states = size(x,2);
                
                P_aug = blkdiag(P,obj.Q,obj.R);
                
                obj.augmented_covariance = P_aug;                 
                obj.augmented_state = [x, 0, 0, 0, 0, 0, 0];
                
                obj.sigma_generation()
                obj.weight_generation()
                
                sigma_points_predict = [];
                
                for i = 1 : size(obj.sigma_points,2)
                    px = obj.sigma_points{i}(1);
                    py = obj.sigma_points{i}(2);
                    psi = obj.sigma_points{i}(3);
                    v = obj.sigma_points{i}(4);
                    yaw_rate_noise = obj.sigma_points{i}(7);
                    accel_noise = obj.sigma_points{i}(8);             

                    px_upd = px + dt * v * cos(psi) + obj.sigma_points{i}(5);
                    py_upd = py + dt * v * sin(psi) + obj.sigma_points{i}(6);
                    psi_upd = psi + (yaw_rate_noise + yaw_rate) * dt ;
                    v_upd = v + accel_noise * dt ;
                    
                    sigma_points_predict{i} = [px_upd, py_upd, psi_upd, v_upd];
                end
                
                x_predict = zeros(1, number_of_states);
                for i = 1 : size(sigma_points_predict,2)
                    x_predict = x_predict+ obj.weights(i) * sigma_points_predict{i};
                end              
                                
                P_predict = zeros(number_of_states,number_of_states);
                
                for i = 1 : size(sigma_points_predict,2)
                    deviation = sigma_points_predict{i} - x_predict;
                    P_predict = P_predict + (obj.weights(i) * (deviation' * deviation));
                end
                
                obj.state = x_predict;
                obj.covariance = P_predict;
            end
        end
        
        function [obj] = update_step_linear(obj, measurement)
            
            if ~isempty(obj.state) && ~isempty(obj.covariance)
                x = obj.state;
                P = obj.covariance;
                H = obj.H;
                R = obj.R;    
                
                z = [measurement(1),measurement(2)];
                z_hat = H * x';
                z_hat = z_hat';
                
                y = z - z_hat;
                S = H*P*H' + R;
                K = P*H'/S;
                
                x_update = x + (K * y')';
                P_update = P - K*H*P;
                
                obj.innovation = y;
                obj.innovation_covariance = S;
                obj.state = x_update;
                obj.covariance = P_update;
                
            else
                obj.state = ([measurement(1), measurement(2),0,0]);
                obj.covariance = diag([obj.R(1,1),obj.R(2,2),0.5,0.5]);
            end
        end
        
        function [obj] = update_step(obj, sight_x, sight_y, range, heading, range_std, heading_std)
            
            if ~isempty(obj.state) && ~isempty(obj.covariance)
                
                x = obj.state;
                
                P = obj.covariance;
                
                z = [range,heading];          
                
                R_polar = diag([range_std*range_std; heading_std*heading_std]);
                
                number_of_states = size(x,2);
                noise_vars = 2;
                
                P_aug = blkdiag(P,obj.Q,R_polar);
                obj.augmented_covariance = P_aug;
                obj.augmented_state = [x, 0, 0, 0, 0, 0, 0];
                obj.sigma_generation()
                
                z_sigma = [];
                
                for i = 1 : size(obj.sigma_points,2)
                    px = obj.sigma_points{i}(1);
                    py = obj.sigma_points{i}(2);
                    psi = obj.sigma_points{i}(3);
                    range_noise = obj.sigma_points{i}(9);
                    heading_noise = obj.sigma_points{i}(10);
                    
                    dx = sight_x - px;
                    dy = sight_y - py;
                
                    line_of_sight = sqrt(dx^2 + dy^2) + range_noise;
                    orientation = atan2(dy,dx) - psi + heading_noise;
                    
                    z_sigma{i} = [line_of_sight, orientation];                    
                end
                
                z_mean = zeros(1,2);
                
                for i = 1 : size(z_sigma,2)
                    z_mean = z_mean + obj.weights(i)* z_sigma{i};
                end
                
                S = zeros(noise_vars,noise_vars);
                
                for i = 1 : size(z_sigma,2)
                    deviation = z_sigma{i} - z_mean;
                    S = S + obj.weights(i) * (deviation' * deviation);
                end
                
                crossCov = zeros(number_of_states,noise_vars);
                
                for i = 1 : 2 * 4 + 2
                    state_dev = obj.sigma_points{i}(1:4) - x;
                    meas_dev = z_sigma{i} - z_mean;
                    crossCov = crossCov + obj.weights(i) * state_dev' * meas_dev;
                end
                
                K = crossCov/S;
                y = z - z_mean;
                x_update = x + (K * y')';
                P_update = P - K*S*K';
                
                obj.innovation = y;
                obj.innovation_covariance = S;
                obj.state = x_update;
                obj.covariance = P_update;
            end
        end
    end
end