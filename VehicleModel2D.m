classdef VehicleModel2D < handle
    
    properties
        x_pos
        y_pos
        yaw
        x_vel
        y_vel
    end
    
    methods
        
        function [obj] = VehicleModel2D()
            obj.x_pos = 0;
            obj.y_pos = 0;
            obj.yaw = 0;
            obj.x_vel = 0;
            obj.y_vel = 0;
        end
              
         function [obj] = initialise(obj,vehicle_params)
            obj.x_pos = vehicle_params.initial_x_position;
            obj.y_pos = vehicle_params.initial_y_position;
            obj.yaw = vehicle_params.initial_heading;
            obj.x_vel = cos(obj.yaw) * vehicle_params.initial_speed;
            obj.y_vel = sin(obj.yaw) * vehicle_params.initial_speed;
        end
        
        function [obj] = update_vehicle(obj,time_step,accel_x,accel_y,yaw_rate)
            obj.x_pos = obj.x_pos + obj.x_vel * time_step + time_step^2/2 * accel_x;
            obj.y_pos = obj.y_pos + obj.y_vel * time_step + time_step^2/2 * accel_y;
            obj.x_vel = obj.x_vel + accel_x * time_step;
            obj.y_vel = obj.y_vel + accel_y * time_step;
            obj.yaw = obj.yaw + yaw_rate * time_step;
        end
    end
end