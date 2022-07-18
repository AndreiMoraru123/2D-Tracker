classdef QuaternionKalmanFilterModel < handle
    
    properties
        F
        H
        Q
        R
        norm
        state
        attitude_state
        measured_state
        rates
        covariance
        innovation
        innovation_covariance
    end
    
    methods
        
        function [obj] = QuaternionKalmanFilterModel(obj)
            obj.state = [];
            obj.covariance = zeros(4,4);
            obj.innovation = [];
            obj.innovation_covariance = [];
        end
        
        function [obj] = quatNorm(obj)
            q0 = obj.state(1);
            q1 = obj.state(2);
            q2 = obj.state(3);
            q3 = obj.state(4);
            obj.norm = sqrt(q0*q0 + q1*q1 + q2 * q2 + q3*q3);
        end
        
        function [obj] = quatFromEuler(obj)
            attitude = obj.attitude_state;
            c1 = cosd(0.5 * attitude(1));
            s1 = sind(0.5 * attitude(1));
            c2 = cosd(0.5 * attitude(2));
            s2 = sind(0.5 * attitude(2));
            c3 = cosd(0.5 * attitude(3));
            s3 = sind(0.5 * attitude(3));
            q0 = c1*c2*c3 + s1*s2*s3;
            q1 = s1*c2*c3 - c1*s2*s3;
            q2 = c1*s2*c3 + s1*c2*s3;
            q3 = c1*c2*s3 - s1*s2*c3;
            
            obj.state = [q0; q1; q2; q3];
        end
        
        
        function [obj] = QuatFromEuler(obj, attitude)
            c1 = cosd(0.5 * attitude(1));
            s1 = sind(0.5 * attitude(1));
            c2 = cosd(0.5 * attitude(2));
            s2 = sind(0.5 * attitude(2));
            c3 = cosd(0.5 * attitude(3));
            s3 = sind(0.5 * attitude(3));
            q0 = c1*c2*c3 + s1*s2*s3;
            q1 = s1*c2*c3 - c1*s2*s3;
            q2 = c1*s2*c3 + s1*c2*s3;
            q3 = c1*c2*s3 - s1*s2*c3;
            
            obj.measured_state = [q0 q1 q2 q3];
            
        end
        
        function [obj] = eulerFromQuat(obj)
            q0 = obj.state(1);
            q1 = obj.state(2);
            q2 = obj.state(3);
            q3 = obj.state(4);
            r11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            r12 = 2*(q1*q2 + q0*q3);
            r13 = 2*(q1*q3 - q0*q2);
            r23 = 2*(q2*q3 + q0*q1);
            r33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            phi = atan2(r23,r33);
            theta = -asin(r13);
            psi = atan2(r12,r11);
            obj.attitude_state = [phi;theta;psi];
        end
        
        function [obj] = quaternionRates(obj, omega_body)
            q0 = obj.state(1);
            q1 = obj.state(2);
            q2 = obj.state(3);
            q3 = obj.state(4);
            W = [-q1, -q2, -q3;
                q0, q3, -q2;
                -q3, q0, q1;
                q2, -q1, q0];
            obj.rates = 0.5 * mtimes(W,omega_body);
        end
        
        function [obj] = eulerIntegration(obj,dt)
            obj.state = obj.state + obj.rates * dt;
        end
        
        function [obj] = normalizeQuat(obj)
            obj.quatNorm();
            obj.state = obj.state/obj.norm;
        end
        
        function [obj] = initialise(obj, q1Std, q2Std, q3Std, q4Std, a1Std, a2Std, a3Std, a4Std, init_on_measurement, init_q1_std, init_q2_std, init_q3_std, init_q4_std, measurement, varargin)
%             obj.H = eye(2,4);
            obj.H = eye(4);
            obj.F = eye(4);
            obj.Q = diag([q1Std*q1Std, q2Std*q2Std, q3Std*q3Std, q4Std*q4Std]);
%             obj.R = diag([a1Std*a1Std, a2Std*a2Std]);
            obj.R = diag([a1Std*a1Std, a2Std*a2Std, a3Std*a3Std, a4Std*a4Std]);
            
            if init_on_measurement == false
                obj.state = [0; 0; 0; 0];
                obj.covariance = diag([init_q1_std*init_q1_std,init_q2_std*init_q2_std,init_q3_std*init_q3_std,init_q4_std*init_q4_std]);
            else
                obj.state = [measurement(1); measurement(2); measurement(3); measurement(4)];
                obj.covariance = diag([init_q1_std*init_q1_std,init_q2_std*init_q2_std,init_q3_std*init_q3_std,init_q4_std*init_q4_std]);
            end
        end
        
        function [obj] = prediction(obj, omega_body, dt)
            
            P = obj.covariance;
            
            obj.quaternionRates(omega_body);
            obj.eulerIntegration(dt);
            
            if ~all(obj.state == 0)
                obj.normalizeQuat();
            end
            
            obj.eulerFromQuat();
            P_predict = obj.F*P*obj.F' + obj.Q;
            obj.covariance = P_predict;
        end
        
        function [obj] = update(obj,measurement)
            
            P = obj.covariance;
            
            X = obj.state';
            
%             obj.eulerFromQuat();
%             
%             X = obj.attitude_state';

            measurement = [measurement; 0];

            obj.QuatFromEuler(measurement);
            
%             z_hat = [X(1) X(2)];

            z_hat = obj.state';

            z = obj.measured_state;
              
%             z = [measurement(1) measurement(2)];
          
            y = z - z_hat;
            S = obj.H*P*obj.H' + obj.R;
            K = P*obj.H'/S;
            
%             x = [X(1), X(2), X(3), 0];
%             
%             x_update = x + (K * y')';
            
            x_update = X + (K * y')';
            P_update = P - K*obj.H*P;
            
            obj.innovation = y;
            obj.innovation_covariance = S;
%             obj.attitude_state = x_update(1:3)';
            
            obj.state = x_update';
            
%             obj.quatFromEuler();
            
            if ~all(obj.state == 0)
                obj.normalizeQuat();
            end
            
            obj.eulerFromQuat()
            
            obj.covariance = P_update;
        end       
    end
end