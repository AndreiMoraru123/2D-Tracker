classdef ExtendedKalmanFilterModel < handle
    
    properties
        jcbF
        jcbH
        H
        Q
        R
        state
        covariance
        innovation
        innovation_covariance
    end
    
    methods
        
        function [obj] = ExtendedKalmanFilterModel()
            obj.state = [];
            obj.covariance = zeros(4,4);
            obj.innovation = [];
            obj.innovation_covariance = [];
        end
        
        function [obj] = initialise(obj, time_step, accel_std, yaw_std, meas_std, init_on_measurement, init_pos_std, init_vel_std, init_yaw_std, measurement, varargin)
            dt = time_step;
            obj.H = [1 0 0 0;
                     0 1 0 0];
            obj.Q = diag([0, 0, dt*dt*yaw_std*yaw_std, dt*dt*accel_std*accel_std]);
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
                px = x(1);
                py = x(2);
                psi = x(3);
                v = x(4);
                
                obj.jcbF = [1 0 -dt*v*sin(psi) dt*cos(psi);
                    0 1 dt*v*cos(psi) dt*sin(psi);
                    0 0 1 0;
                    0 0 0 1];
                
                P = obj.covariance;
                
                px_upd = px + time_step * v * cos(psi);
                py_upd = py + time_step * v * sin(psi);
                psi_upd = psi + yaw_rate * time_step;
                v_upd = v;
                
                x_predict = [px_upd py_upd psi_upd v_upd];
                P_predict = obj.jcbF * P * obj.jcbF' + obj.Q;
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
                
                dx = sight_x - x(1);
                dy = sight_y - x(2);
                
                line_of_sight = sqrt(dx^2 + dy^2);
                orientation = atan2(dy,dx) - x(3);
                
                obj.jcbH = [-dx/line_of_sight -dy/line_of_sight 0 0;
                    (dy/(line_of_sight)^2) (-dx/(line_of_sight)^2) -1 0];
                
                Rex = diag([range_std*range_std; heading_std*heading_std]);
                
                z_hat = [line_of_sight, orientation];
                
                y = z - z_hat;
                S = obj.jcbH*P*obj.jcbH' + Rex;
                K = P*obj.jcbH'/S;
                
                x_update = x + (K * y')';
                P_update = P - K*obj.jcbH*P;
                
                obj.innovation = y;
                obj.innovation_covariance = S;
                obj.state = x_update;
                obj.covariance = P_update;
            end
        end
    end
end