classdef UnscentedVehicleModel2D < handle
    
    properties
        x_pos
        y_pos
        yaw
        vel
        yaw_rate_noise
        acc_noise
    end
    
    methods
        
        function [obj] = UnscentedVehicleModel2D()
            obj.x_pos = 0;
            obj.y_pos = 0;
            obj.yaw = 0;
            obj.vel = 0;
            obj.yaw_rate_noise = 0;
            obj.acc_noise = 0;
        end
        
        function [obj] = initialise(obj,vehicle_params)
            obj.x_pos = vehicle_params.initial_x_position;
            obj.y_pos = vehicle_params.initial_y_position;
            obj.yaw = vehicle_params.initial_heading;
            obj.vel = vehicle_params.initial_speed;
            obj.yaw_rate_noise = vehicle_params.heading_noise;
            obj.acc_noise = vehicle_params.acc_noise;
        end
        
        function [obj] = update_vehicle(obj,time_step,accel,yaw_rate)
            obj.x_pos = obj.x_pos + obj.vel * cos(obj.yaw) * time_step;
            obj.y_pos = obj.y_pos + obj.vel * sin(obj.yaw) * time_step;
            obj.yaw = obj.yaw + (yaw_rate + obj.yaw_rate_noise) * time_step;
            obj.vel = obj.vel + (accel + obj.acc_noise) * time_step;
        end
    end
end