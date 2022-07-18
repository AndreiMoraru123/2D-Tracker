classdef ExtendedVehicleModel2D < handle
    
    properties
        x_pos
        y_pos
        yaw
        vel
    end
    
    methods
        
        function [obj] = ExtendedVehicleModel2D()
            obj.x_pos = 0;
            obj.y_pos = 0;
            obj.yaw = 0;
            obj.vel = 0;
        end
        
        function [obj] = initialise(obj,vehicle_params)
            obj.x_pos = vehicle_params.initial_x_position;
            obj.y_pos = vehicle_params.initial_y_position;
            obj.yaw = vehicle_params.initial_heading;
            obj.vel = vehicle_params.initial_speed;
        end
        
        function [obj] = update_vehicle(obj,time_step,accel,yaw_rate)
            obj.x_pos = obj.x_pos + obj.vel * cos(obj.yaw) * time_step;
            obj.y_pos = obj.y_pos + obj.vel * sin(obj.yaw) * time_step;
            obj.yaw = obj.yaw + yaw_rate * time_step;
            obj.vel = obj.vel + accel * time_step;
        end
    end
end