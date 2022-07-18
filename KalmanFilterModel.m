classdef KalmanFilterModel < handle
    
    properties
        F
        H
        Q
        R
        G
        state
        covariance
        innovation
        innovation_covariance
    end
    
    methods
        
        function [obj] = KalmanFilterModel()
            obj.state = [];
            obj.covariance = zeros(4,4);
            obj.innovation = [];
            obj.innovation_covariance = [];
        end
        
        function [obj] = initialise(obj,time_step,accel_std,meas_std,init_on_measurement, init_pos_std, init_vel_std, measurement, varargin)
            dt = time_step;
            obj.F = [1 0 dt 0;
                0 1 0 dt;
                0 0 1 0;
                0 0 0 1];
            obj.H = [1 0 0 0;
                0 1 0 0];
            obj.G = [0.5*dt*dt, 0;
                0, 0.5*dt*dt;
                dt 0;
                0 dt];
            obj.Q = diag([0.5*dt*dt,0.5*dt*dt,dt,dt]) * (accel_std * accel_std);
            obj.R = diag([meas_std*meas_std,meas_std*meas_std]);
            
            if init_on_measurement == false
                obj.state = [0 0 0 0];
                obj.covariance = diag([init_pos_std*init_pos_std, init_pos_std*init_pos_std, init_vel_std*init_vel_std, init_vel_std*init_vel_std]);
            else
                obj.state = [measurement(1) measurement(2) 0 0];
                obj.covariance = diag([meas_std*meas_std, meas_std*meas_std, init_vel_std*init_vel_std, init_vel_std*init_vel_std]); 
            end
        end
        
        function [obj] = prediction_step(obj)
            if ~isempty(obj.state)
                x = obj.state;
                P = obj.covariance;
                
                x_predict = obj.F * x';
                x_predict = x_predict';
                P_predict = obj.F * P * obj.F' + obj.Q;
                obj.state = x_predict;
                obj.covariance = P_predict;
            end
        end
        
        function [obj] = forced_prediction_step(obj,acc)
            if ~isempty(obj.state)
                
                x = obj.state;
                P = obj.covariance;
                       
                x_predict = obj.F * x' + obj.G * acc;
                x_predict = x_predict';
                P_predict = obj.F * P * obj.F' + obj.Q;
                obj.state = x_predict;
                obj.covariance = P_predict;
            end
        end
        
        function [obj] = update_step(obj,measurement)
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
                obj.covariance = diag([obj.R(1,1),obj.R(2,2),10,10]);
            end
        end
    end  
end