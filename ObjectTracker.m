clear; clc; close all;

[dev_mode, filter_type, traj, run_mode, sonar, forced] = TrackerUI();

% cd C:\Users\Andrei\Desktop\Licenta+Lucru\Personale\KalmanFilter\MatlabTracker\ObjectTracking\
% d = System.IO.File.GetCreationTime(cd);
% file = dir('ObjectTracker.m');
% creationDateTime = datetime(d.Year, d.Month, d.Day, d.Hour, d.Minute, d.Second);
% fprintf('Welcome to the 2D Shark Chase Simulator \n \n');
% fprintf('Author: Moraru Andrei \n');
% fprintf('Created on: %s \n', datestr(creationDateTime));
% fprintf('Last modified: %s \n\n', file.date);

if strcmpi(run_mode,'cmdrive') == 1
    try
        port = serialportlist;
        ard = arduino(port,'Nano3');
        fs = 200;
        mpu = mpu6050(ard,'SampleRate',fs);
        ard_kalman_filter = QuaternionKalmanFilterModel();
        phi = 0;
        theta = 0;
        psi = 0;
        ard_Q = 9 * eye(4);
        ard_R = 0.09 * eye(4);
        ard_P = 9 * eye(4);
        ard_q1Std = ard_Q(1,1); ard_q2Std = ard_Q(2,2); ard_q3Std = ard_Q(3,3); ard_q4Std = ard_Q(4,4);
        ard_a1Std = ard_R(1,1); ard_a2Std = ard_R(2,2); ard_a3Std = ard_R(3,3); ard_a4Std = ard_R(4,4);
        ard_init_q1_std = ard_P(1,1); ard_init_q2_std = ard_P(2,2); ard_init_q3_std = ard_P(3,3); ard_init_q4_std = ard_P(4,4);
        ard_init_on_measurement = false;
        ard_dt = 1/fs;
        traj = 'random';
        
    catch ConnectionError
        run_mode ='simu';
    end
    
    
end

if dev_mode == 1
    
    while 1
        prompt = 'Pick your difficulty level: [L/E/U] ';
        filter_type = input(prompt,'s');
        
        if (filter_type == 'E' || filter_type == 'e')
            break;
        end
        
        if (filter_type == 'L' || filter_type == 'l')
            break;
        end
        
        if (filter_type == 'U' || filter_type == 'u')
            break;
        end
        
        if (filter_type ~= 'L' && filter_type ~= 'l' && filter_type ~= 'E' && filter_type ~= 'e'  && filter_type ~= 'U' && filter_type ~= 'u')
            disp('Not a viable choice')
        end
    end
    
    fprintf('\n')
    
    while 1
        prompt = 'Would you like to make changes to the model? [Y/N] ';
        str = input(prompt,'s');
        
        if (str == 'Y' || str == 'y')
            break;
        end
        
        if (str == 'N' || str == 'n')
            initial_x_position = 0;
            initial_y_position = initial_x_position;
            initial_heading = 45;
            initial_speed = 4000;
            accel_std = 20;
            meas_std = 10;
            init_pos_std = 5;
            init_vel_std = 0.5;
            init_yaw_std = 0.005;
            yaw_std = 0.0095;
            init_on_measurement = false;
            max_blood_units = 100;
            motion_type = 'linear';
            break;
        end
        
        if (str ~= 'N' && str ~= 'Y' && str ~= 'n' && str ~= 'y')
            disp('Not a viable choice')
        end
    end
    fprintf('\n')
    
end

if (filter_type == 'L' || filter_type == 'l')
    vehicle_model = VehicleModel2D();
    chaser_model = VehicleModel2D();
    kalman_filter = KalmanFilterModel();
    seagull = 0;
end

if (filter_type == 'E' || filter_type == 'e')
    vehicle_model = ExtendedVehicleModel2D();
    chaser_model = VehicleModel2D();
    kalman_filter = ExtendedKalmanFilterModel();
end

if (filter_type == 'U' || filter_type == 'u')
    vehicle_model = UnscentedVehicleModel2D();
    chaser_model = VehicleModel2D();
    kalman_filter = UnscentedKalmanFilterModel();
end

if dev_mode == 1
    
    if (str == 'Y' || str == 'y')
        
        while 1
            prompt = 'Set the initial position of the seal on X axis: ';
            initial_x_position = input(prompt);
            if (~isnumeric(initial_x_position))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the initial position of the seal on Y axis: ';
            initial_y_position = input(prompt);
            if (~isnumeric(initial_y_position))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the initial speed of the seal [m/s]: ';
            initial_speed = input(prompt);
            if (initial_speed < 1000)
                initial_speed = initial_speed * 100;
            end
            if (~isnumeric(initial_speed))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the initial heading of the seal [degrees]: ';
            initial_heading = input(prompt);
            if (~isnumeric(initial_heading))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set maximum blood loss units available for the seal: ';
            max_blood_units = input(prompt);
            if (~isnumeric(initial_x_position))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the motion type: [linear / circle / random] ';
            motion_type = input(prompt);
            fprintf('\n')
            if (~isstring(motion_type) && ~ischar(motion_type))
                disp("That's not a good idea")
            else
                break;
            end
        end
    end
    
    while 1
        prompt = 'Proceed to tune the filter? [Y/N] ';
        str = input(prompt,'s');
        
        if (str == 'Y' || str == 'y')
            break;
        end
        
        if (str == 'N' || str == 'n')
            accel_std = 5;
            meas_std = 10;
            init_pos_std = 5;
            init_vel_std = 0.5;
            init_yaw_std = 0.005;
            yaw_std = 0.005;
            init_on_measurement = false;
            break;
        end
        
        if (str ~= 'N' && str ~= 'Y' && str ~= 'n' && str ~= 'y')
            disp('Not a viable choice')
        end
    end
    
    
    if (str == 'Y' || str == 'y')
        
        while 1
            prompt = 'Set the acceleration standard deviation [m/s^2]: ';
            accel_std = input(prompt);
            if (~isnumeric(accel_std))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            
            while 1
                prompt = 'Set the yaw standard deviation [degrees]: ';
                yaw_std = deg2rad(input(prompt));
                if (~isnumeric(yaw_std))
                    disp("That's not a good idea")
                else
                    break;
                end
            end
        end
        
        while 1
            
            prompt = 'Set the measurement standard deviation [m]: ';
            meas_std = input(prompt);
            if (~isnumeric(meas_std))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the initial position standard deviation [m]: ';
            init_pos_std = input(prompt);
            if (~isnumeric(init_pos_std))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the initial velocity standard deviation [m/s]: ';
            init_vel_std = input(prompt);
            if (~isnumeric(init_vel_std))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            
            while 1
                prompt = 'Set the initial yaw standard deviation [degrees]: ';
                init_yaw_std = deg2rad(input(prompt));
                if (~isnumeric(init_yaw_std))
                    disp("That's not a good idea")
                else
                    break;
                end
            end
        end
        
        if (filter_type == 'L' || filter_type == 'l')
            while 1
                prompt = 'Start the shark chase on the first measurement? [True/False]: ';
                init_on_measurement = input(prompt);
                if (~islogical(init_on_measurement))
                    disp("That's not a good idea")
                else
                    break;
                end
            end
        end
    end
    fprintf('\n')
    
    if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
        init_on_measurement = true;
        
        while 1
            prompt = 'Activate seagull? [True/False]: ';
            seagull = input(prompt);
            if (~islogical(seagull))
                disp("That's not a good idea")
            else
                break;
            end
        end
    end
    
    if (seagull == 1)
        
        while 1
            prompt = 'Set the range standard deviation [m]: ';
            sgl_range_std = input(prompt);
            
            if (~isnumeric(sgl_range_std))
                disp("That's not a good idea")
            else
                break;
            end
        end
        
        while 1
            prompt = 'Set the heading standard deviation [degrees]: ';
            sgl_heading_std = deg2rad(input(prompt));
            
            if (~isnumeric(sgl_heading_std))
                disp("That's not a good idea")
            else
                break;
            end
        end
    end
    
end

if dev_mode == 0
    
    initial_x_position = 50;
    initial_y_position = initial_x_position;
    initial_heading = 45;
    initial_speed = 4500;
    accel_std = 20;
    meas_std = 10;
    init_pos_std = 5;
    init_vel_std = 0.5;
    init_yaw_std = 0.0095;
    yaw_std = 0.005;
    init_on_measurement = false;
    max_blood_units = 120;
    
    seagull = sonar;
    
    if seagull
        sgl_range_std = 10;
        sgl_heading_std = deg2rad(0.005);
    end
    
    motion_type = traj;
    
end

fprintf('\n')

measurement_noise_std = 300;
heading_noise = randn() * deg2rad(0.05);
acc_noise = randn() * 20;

vehicle_params = struct('initial_x_position',initial_x_position,'initial_y_position',initial_y_position,...
    'initial_heading', deg2rad(initial_heading),'initial_speed',initial_speed, 'heading_noise', heading_noise, 'acc_noise',acc_noise);

vehicle_model.initialise(vehicle_params);

initial_measurement = [vehicle_model.x_pos + randn() * measurement_noise_std;...
    vehicle_model.x_pos + randn() * measurement_noise_std];

if init_on_measurement
    
    chaser_params = struct('initial_x_position',initial_measurement(1),'initial_y_position',initial_measurement(2),...
        'initial_heading', 0,'initial_speed',0, 'heading_noise', heading_noise, 'acc_noise',acc_noise);
    
else
    chaser_params = struct('initial_x_position',0,'initial_y_position',0,...
        'initial_heading', 0,'initial_speed',0, 'heading_noise', heading_noise, 'acc_noise',acc_noise);
end

chaser_model.initialise(chaser_params);

if (filter_type == 'L' || filter_type == 'l')
    kf_options = struct('accel_std',accel_std,'meas_std',meas_std,'init_on_measurement',init_on_measurement,...
        'init_pos_std',init_pos_std,'init_vel_std',init_vel_std,'initial_measurement',initial_measurement);
end

if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
    ekf_options = struct('accel_std',accel_std,'yaw_std',yaw_std,'meas_std',meas_std,...
        'init_on_measurement',init_on_measurement, 'init_pos_std',init_pos_std,...
        'init_vel_std',init_vel_std,'init_yaw_std',init_yaw_std,'initial_measurement',initial_measurement);
end

f = figure;
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'color',[0 0.4470 0.7410]);
set(gcf,'color',[0 0.4470 0.7410]);

[Seal,X,Y] = SealDrawer();
[Shark,Xs,Ys] = SharkDrawer();

if seagull
    [Seagull,Xl,Yl] = SeagullDrawer();
    
    for index = 1:length(Seagull)
        Xl{index} = 300 * Xl{index};
        Yl{index} = 300 * Yl{index};
    end
end

for index = 1:length(Seal)
    X{index} = 500 * X{index};
    Y{index} = 500 * Y{index};
end

for index = 1:length(Shark)
    Xs{index} = 400 * Xs{index};
    Ys{index} = 400 * Ys{index};
end

dt = 0.15;
i = 1;
axis off ;
blood = 0;
counter_seconds = 0;
et = num2str(counter_seconds);
elapsed_time = annotation('textbox', [0, 0.5, 0, 0], 'string', et, 'FontSize', 20, 'Color', 'w');

innovation_history_x = [];
innovation_history_y = [];

chaser_error_history = [];

estimation_error_history = [];
estimated_covariance_history_x_pos = [];
estimated_covariance_history_y_pos = [];
estimated_covariance_history_x_vel = [];
estimated_covariance_history_y_vel = [];
estimated_covariance_history_vel = [];
estimated_covariance_history_yaw = [];

vehicle_position_history_x = [];
vehicle_position_history_y = [];

vehicle_velocity_history_x = [];
vehicle_velocity_history_y = [];

vehicle_heading_history = [];
vehicle_velocity_history = [];

chaser_position_history_x = [];
chaser_position_history_y = [];

chaser_velocity_history_x = [];
chaser_velocity_history_y = [];

chaser_acceleration_history_x = [];
chaser_acceleration_history_y = [];

vehicle_acceleration_history_x = [];
vehicle_acceleration_history_y = [];

measurement_history_x = [];
measurement_history_y = [];

estimated_state_history_x_pos = [];
estimated_state_history_y_pos = [];
estimated_state_history_x_vel = [];
estimated_state_history_y_vel = [];
estimated_state_history_vel = [];
estimated_state_history_yaw = [];

MP=get(0,'MonitorPositions');
if size(MP,1)>1
    pos=get(f,'Position');
    set(f,'Position',[pos(1,2)+MP(2,1:2) pos(3:4)]);
end

N = size(MP,1);
newPosition = MP(1,:);

if size(MP,1) == 1
    
else
    newPosition(1) = newPosition(1) + MP(N,1);
end

f.set('Position', newPosition, 'units', 'normalized');
f.WindowState = 'maximized';


sgl = 0;
draw_sgl = 1;
kappa = 0;

axis tight

while ishandle(f)
    
    if strcmpi(run_mode,'cmdrive') == 1
        matr = table2array((read(mpu)));
        
        accel = matr(1:end,1:3);
        gyro = matr(1:end,4:6);
        
        acc_x = accel(end,1);
        acc_y = accel(end,2);
        acc_z = accel(end,3);
        
        gx = rad2deg(gyro(end,2));
        gy = rad2deg(gyro(end,1));
        gz = rad2deg(gyro(end,3));
        
        gyro_x = gx + gy * tand(theta) * sind(phi) + gz * tand(theta) * cosd(phi);
        gyro_y = gy * cosd(phi) - gz * sind(phi);
        gyro_z = gz;
        
        omega_body = ([gyro_x; gyro_y; gyro_z]);
        
        acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
        angle_Roll_acc = (asind(double(acc_x)/acc_total_vector));
        angle_Pitch_acc = (asind(double(acc_y)/acc_total_vector));
        accel_angle = [angle_Roll_acc; angle_Pitch_acc];
        
        yaw_rate = deg2rad(gx*0.05  - 20);
        
    end
    
    if strcmpi(run_mode,'simu') == 1
        yaw_rate = deg2rad(randn()*0.05  - 20);
        acc_x = randn() * 20;
        acc_y = randn() * 1.5 * 10;
    end
    
    if (filter_type == 'L' || filter_type == 'l')
        vehicle_acceleration_history_x = [vehicle_acceleration_history_x, acc_x];
        vehicle_acceleration_history_y = [vehicle_acceleration_history_x, acc_y];
    else
        vehicle_acceleration_history_x = [vehicle_acceleration_history_x, acc_x * cos(vehicle_model.yaw)];
        vehicle_acceleration_history_y = [vehicle_acceleration_history_x, acc_x * sin(vehicle_model.yaw)];
    end
    
    
    if i == 1
        
        if strcmpi(run_mode,'cmdrive') == 1
            ard_kalman_filter.initialise(ard_q1Std, ard_q2Std, ard_q3Std, ard_q4Std, ard_a1Std, ard_a2Std, ard_a3Std, ard_a4Std,...
                ard_init_on_measurement, ard_init_q1_std, ard_init_q2_std, ard_init_q3_std, ard_init_q4_std);
            Roll = 0;
            Pitch = 0;
        end
        
        if (filter_type == 'L' || filter_type == 'l')
            kalman_filter.initialise(dt,kf_options.accel_std,kf_options.meas_std,kf_options.init_on_measurement,...
                kf_options.init_pos_std,kf_options.init_vel_std,kf_options.initial_measurement);
        end
        
        if (filter_type == 'E' || filter_type == 'e')
            kalman_filter.initialise(dt,ekf_options.accel_std,ekf_options.yaw_std,ekf_options.meas_std,...
                ekf_options.init_on_measurement,ekf_options.init_pos_std,ekf_options.init_vel_std,...
                ekf_options.init_yaw_std,ekf_options.initial_measurement);
        end
        
        if (filter_type == 'U' || filter_type == 'u')
            kalman_filter.initialise(ekf_options.accel_std,ekf_options.yaw_std,ekf_options.meas_std,...
                ekf_options.init_on_measurement,ekf_options.init_pos_std,ekf_options.init_vel_std,...
                ekf_options.init_yaw_std,ekf_options.initial_measurement);
        end
        
        
    else
        
        if strcmpi(run_mode,'cmdrive') == 1
            ard_kalman_filter.prediction(omega_body,ard_dt);
            ard_kalman_filter.update(accel_angle);
            real_angle = ard_kalman_filter.attitude_state;
            
            phi = (real_angle(1));
            theta = (real_angle(2));
            psi = (real_angle(3));
            
            Roll = -phi;
            Pitch = -theta;
            
            if i == 2 
                bias_Roll = Roll;
                bias_Pitch = Pitch;
            end
        end
        
        if strcmpi(motion_type,'linear') == 1
            yaw_rate = 0;
            if (filter_type == 'L' || filter_type == 'l')
                vehicle_model.update_vehicle(dt,abs(acc_x),abs(acc_y),yaw_rate);
            else
                vehicle_model.update_vehicle(dt,abs(acc_x),yaw_rate)
            end
        else
            if (filter_type ~= 'L' && filter_type ~= 'l')
                vehicle_model.update_vehicle(dt,abs(acc_x),yaw_rate);
            end
        end
        
        for j = 1:length(Seal)
            Seal(j).YData = Y{j} + vehicle_model.y_pos;
            Seal(j).XData = X{j} + vehicle_model.x_pos;
        end
        
        mx = vehicle_model.x_pos + randn() * measurement_noise_std;
        my = vehicle_model.y_pos + randn() * measurement_noise_std;
        
        if (filter_type == 'L' || filter_type == 'l')
            if forced == 0
                kalman_filter.prediction_step();
            else
                kalman_filter.forced_prediction_step([acc_x;acc_x]);
            end
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            kalman_filter.prediction_step(dt, yaw_rate + randn()*0.05);
        end
        
        measurement = [mx,my];
        watch_point = 4500;
        watch_point_x = watch_point + measurement(1);
        watch_point_y = watch_point + measurement(2);
        
        measurement_history_x = [measurement_history_x,measurement(1)];
        measurement_history_y = [measurement_history_y,measurement(2)];
        
        vehicle_position_history_x = [vehicle_position_history_x,vehicle_model.x_pos];
        vehicle_position_history_y = [vehicle_position_history_y,vehicle_model.y_pos];
        
        if (filter_type == 'L' || filter_type == 'l')
            vehicle_velocity_history_x = [vehicle_velocity_history_x,vehicle_model.x_vel];
            vehicle_velocity_history_y = [vehicle_velocity_history_y,vehicle_model.y_vel];
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            vehicle_heading_history = [vehicle_heading_history,vehicle_model.yaw];
            vehicle_velocity_history = [vehicle_velocity_history,vehicle_model.vel];
        end
        
        if (filter_type == 'L' || filter_type == 'l')
            kalman_filter.update_step(measurement)
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            kalman_filter.update_step_linear(measurement)
            
            if sgl > 0 && seagull == 1
                dx = sgl_data_x - vehicle_model.x_pos;
                dy = sgl_data_y - vehicle_model.y_pos;
                polar_sight = sqrt(dx^2 + dy^2);
                orientation =(atan2(dy,dx) - vehicle_model.yaw);
                kalman_filter.update_step(sgl_data_x, sgl_data_y, polar_sight, orientation, sgl_range_std , sgl_heading_std);
            end
        end
        
        estimated_state_history_x_pos = [estimated_state_history_x_pos,kalman_filter.state(1)];
        estimated_state_history_y_pos = [estimated_state_history_y_pos,kalman_filter.state(2)];
        
        if (filter_type == 'L' || filter_type == 'l')
            estimated_state_history_x_vel = [estimated_state_history_x_vel,kalman_filter.state(3)];
            estimated_state_history_y_vel = [estimated_state_history_y_vel,kalman_filter.state(4)];
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            estimated_state_history_yaw = [estimated_state_history_yaw,kalman_filter.state(3)];
            estimated_state_history_vel = [estimated_state_history_vel,kalman_filter.state(4)];
        end
        
        estimated_covariance_history_x_pos  = [estimated_covariance_history_x_pos,kalman_filter.covariance(1,1)];
        estimated_covariance_history_y_pos  = [estimated_covariance_history_y_pos,kalman_filter.covariance(2,2)];
        
        if (filter_type == 'L' || filter_type == 'l')
            estimated_covariance_history_x_vel  = [estimated_covariance_history_x_vel,kalman_filter.covariance(3,3)];
            estimated_covariance_history_y_vel  = [estimated_covariance_history_y_vel,kalman_filter.covariance(4,4)];
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            estimated_covariance_history_yaw  = [estimated_covariance_history_yaw,kalman_filter.covariance(3,3)];
            estimated_covariance_history_vel  = [estimated_covariance_history_vel,kalman_filter.covariance(4,4)];
        end
        
        
        A = chaser_model.x_pos;
        B = vehicle_model.x_pos;
        C = chaser_model.y_pos;
        D = vehicle_model.y_pos;
        
        if (filter_type == 'L' || filter_type == 'l')
            estimation_error = [kalman_filter.state(1) - vehicle_model.x_pos,...
                kalman_filter.state(2) - vehicle_model.y_pos,...
                kalman_filter.state(3) - vehicle_model.x_vel,...
                kalman_filter.state(4) - vehicle_model.y_vel];
            estimation_error_history = [estimation_error_history,estimation_error];
            
            chaser_error = [chaser_model.x_pos - kalman_filter.state(1),...
                chaser_model.y_pos - kalman_filter.state(2),...
                chaser_model.x_vel - kalman_filter.state(3),...
                chaser_model.y_vel - kalman_filter.state(4)];
            chaser_error_history = [chaser_error_history,chaser_error];
            
            Gain = dlqr(kalman_filter.F, kalman_filter.G, 99*eye(4), 0.99 * eye(2));
            input_acc = - Gain * chaser_error';
            
            chaser_model.update_vehicle(dt,input_acc(1),input_acc(2),0);
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            estimation_error = [kalman_filter.state(1) - vehicle_model.x_pos,...
                kalman_filter.state(2) - vehicle_model.y_pos,...
                kalman_filter.state(3) - vehicle_model.yaw,...
                kalman_filter.state(4) - vehicle_model.vel];
            estimation_error_history = [estimation_error_history,estimation_error];
            
            chaser_error = [chaser_model.x_pos - kalman_filter.state(1),...
                chaser_model.y_pos - kalman_filter.state(2),...
                chaser_model.x_vel - kalman_filter.state(4) * cos(kalman_filter.state(3)),...
                chaser_model.y_vel - kalman_filter.state(4) * sin(kalman_filter.state(3))];
        
            F = [1 0 dt 0;
                0 1 0 dt;
                0 0 1 0;
                0 0 0 1];
            
            G = [0.5*dt*dt, 0;
                0, 0.5*dt*dt;
                dt 0;
                0 dt];         

%             Gain = dlqr(F, G, 9999*eye(4), 0.99 * eye(2));
            Gain = place(F,G,[0.1 0.2 0.25 0.15]/100);
            input_acc = - Gain * chaser_error';
            
            chaser_model.update_vehicle(dt,input_acc(1),input_acc(2),0);
        end
        
        chaser_position_history_x = [chaser_position_history_x,chaser_model.x_pos];
        chaser_position_history_y = [chaser_position_history_y,chaser_model.y_pos];
        
        chaser_velocity_history_x = [chaser_velocity_history_x,chaser_model.x_vel];
        chaser_velocity_history_y = [chaser_velocity_history_y,chaser_model.y_vel];
        
        chaser_acceleration_history_x = [chaser_acceleration_history_x,input_acc(1)];
        chaser_acceleration_history_y = [chaser_acceleration_history_y,input_acc(2)];
        
        for j = 1:length(Shark)
            Shark(j).XData = Xs{j} + chaser_model.x_pos;
            Shark(j).YData = Ys{j} + chaser_model.y_pos;
        end
        
        if ~isempty(kalman_filter.innovation)
            innovation_history_x = [innovation_history_x,kalman_filter.innovation(1)];
            innovation_history_y = [innovation_history_y,kalman_filter.innovation(2)];
        end
        
        if norm([A C] - [B D]) < 200
            plot(mx,my,'.r','MarkerSize',40)
            blood = blood + 1;
        else
            plot(mx,my,'c+','linew',5)
        end
        
        pause(0.1)
        
        if strcmpi(motion_type,'random') == 1
            
            if strcmpi(run_mode,'cmdrive') == 1
                
                if (vehicle_model.x_pos > min(axis) && vehicle_model.x_pos < max(axis) &&...
                        vehicle_model.y_pos > min(axis) && vehicle_model.y_pos < max(axis))
                    vehicle_model.yaw =  deg2rad(kappa) +  Roll + bias_Roll;
                end
                
                if rad2deg(Pitch) > 20
                    kappa = 0;
                end
                
                if rad2deg(Pitch) < -20
                    kappa = 180;
                end
                
            end
            
            if strcmpi(run_mode,'simu') == 1
                
                if (vehicle_model.x_pos > min(axis) && vehicle_model.x_pos < max(axis) &&...
                        vehicle_model.y_pos > min(axis) && vehicle_model.y_pos < max(axis))
                    vehicle_model.yaw = vehicle_model.yaw + rand() - 1/2;
                end
            end
            
        end
        
        if (blood == max_blood_units)
            delete(findobj('type', 'patch'));
            txt = 'Done';
            text(1000,1000,txt,'FontSize',20,'Color','w')
            try
                w = waitforbuttonpress;
                if(w == 1)
                    close all
                    break;
                else
                    close all
                    break;
                end
            catch
            end
            delete(elapsed_time)
            et = num2str(counter_seconds);
            elapsed_time = annotation('textbox', [0, 1, 0, 0], 'string', et,'FontSize',20,'Color','w');
        end
        
        counter_seconds = counter_seconds + 0.25;
        
        if mod(i,2) == 1 && floor(counter_seconds) == counter_seconds
            et = num2str(counter_seconds);
            elapsed_time = annotation('textbox', [0, 1, 0, 0], 'string', et,'FontSize',20,'Color','w');
        end
        
        if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
            
            if seagull == 1
                
                sgl = sgl + 1;
                
                if draw_sgl == 1
                    
                    right_wing_x = [0 0.5 1] * 500 - watch_point_x;
                    right_wing_y = [0 0.5 0] * 500 - watch_point_y;
                    left_wing_x = [-1 -0.5 0] * 500 - watch_point_x;
                    left_wing_y = [0 0.5 0] * 500 - watch_point_y;
                    sgl_data_x = right_wing_x(1);
                    sgl_data_y = right_wing_y(1);
                    
                    draw_sgl = 0;
                    
                    for j = 1:length(Seagull)
                        Seagull(j).YData = Yl{j} + sgl_data_y;
                        Seagull(j).XData = Xl{j} + sgl_data_x;
                    end
                end
                vision(sgl) = plot([right_wing_x(3),vehicle_model.x_pos],[right_wing_y(3), vehicle_model.y_pos],'y--','linew',2.5);
                if mod(sgl,2) == 0
                    if exist('vision')
                        set(vision,'Visible','off')
                    end
                end
            end
        end
        
        if mod(i,2) == 0
            delete(elapsed_time)
        end
    end
    i = i + 1;
end
close all

innovation_covariance_history_x = ones(length(innovation_history_x),1) * kalman_filter.innovation_covariance(1,1);
innovation_covariance_history_y = ones(length(innovation_history_y),1) * kalman_filter.innovation_covariance(2,2);

while 1
    prompt = 'Proceed to analyze filter performance? [Y/N] ';
    str = input(prompt,'s');
    
    if (str == 'Y' || str == 'y')
        fig = figure('NumberTitle', 'off', 'Name', 'Innovation Analyzer');
        
        if size(MP,1)>1
            pos=get(fig,'Position');
            set(fig,'Position',[pos(1,2)+MP(2,1:2) pos(3:4)]);
        end
        
        if size(MP,1) == 1
        else
            newPosition(1) = newPosition(1) + MP(N,1);
        end
        
        fig.set('Position', newPosition, 'units', 'normalized');
        fig.WindowState = 'maximized';
        a01 = subplot(2,2,1);
        plot(innovation_history_x / 100,'linew',1.5,'color',[0.560000 0.930000 0.560000])
        hold on
        plot([0, length(innovation_history_x)],[0,0],'r--','linew',1.5)
        plot(1*sqrt(innovation_covariance_history_x),'--','linew',1.5,'color',[0.980000 0.840000 0.650000])
        plot(-1*sqrt(innovation_covariance_history_x),'--','linew',1.5,'color',[0.980000 0.840000 0.650000])
        axis tight
        grid on
        a03 = subplot(2,2,3);
        plot(innovation_history_y / 100,'linew',1.5,'color',[0.560000 0.930000 0.560000])
        hold on
        plot([0, length(innovation_history_y)],[0,0],'r--','linew',1.5)
        plot(1*sqrt(innovation_covariance_history_y) ,'--','linew',1.5,'color',[0.980000 0.840000 0.650000])
        plot(-1*sqrt(innovation_covariance_history_y),'--','linew',1.5,'color',[0.980000 0.840000 0.650000])
        axis tight
        grid on 
        a02 = subplot(2,2,2);
        h1 = histfit(chaser_acceleration_history_x/100); alpha(0.5);
        h1(2).Color = 'b';
        hold on
        histfit(vehicle_acceleration_history_x); alpha(0.5);
        axis off
        axis tight
        a04 = subplot(2,2,4);
        h2 = histfit(chaser_acceleration_history_y/100); alpha(0.5);
        h2(2).Color = 'b';
        hold on
        histfit(vehicle_acceleration_history_y); alpha(0.5);
        axis off
        axis tight
        legend(a01,'inovatie','nul (0)','varianta','location','westoutside','fontsize',14)
        legend(a04,'comanda','urmaritor','acceleratie','proces','location','eastoutside','fontsize',14)
        break;
    end
    
    if (str == 'N' || str == 'n')
        break;
    end
    
    if (str ~= 'N' && str ~= 'Y' && str ~= 'n' && str ~= 'y')
        disp('Not a viable choice')
        break;
    end
end

if (str == 'Y' || str == 'y')
    fig = figure('NumberTitle', 'off', 'Name', 'Performance Analyzer');
    
    if size(MP,1)>1
        pos=get(fig,'Position');
        set(fig,'Position',[pos(1,2)+MP(2,1:2) pos(3:4)]);
    end
    
    if size(MP,1) == 1
    else
        newPosition(1) = newPosition(1) + MP(N,1);
    end
    
    fig.set('Position', newPosition, 'units', 'normalized');
    fig.WindowState = 'maximized';
    ax1 = subplot(4,2,1);
    plot(vehicle_position_history_x,'color',[0 0.4470 0.7410],'linew',1.5)
    hold on
    plot(chaser_position_history_x,'color',[0.9290 0.6940 0.1250],'linew',1.5)
    plot(estimated_state_history_x_pos,'color',[0.4660 0.6740 0.1880],'linew',1.5)
    plot(measurement_history_x,'x','color',[0.200000 0.080000 0.080000],'Markersize',10)
    title('Pozitie X');
    grid on;
    axis tight
    ax1.Position = ax1.Position + [0 0 0.05 0.05];
    ax2 = subplot(4,2,2);
    plot(vehicle_position_history_y,'color',[0 0.4470 0.7410],'linew',1.5)
    hold on
    plot(chaser_position_history_y,'color',[0.9290 0.6940 0.1250],'linew',1.5)
    plot(estimated_state_history_y_pos,'color',[0.4660 0.6740 0.1880],'linew',1.5)
    plot(measurement_history_y,'x','color',[0.200000 0.080000 0.080000],'Markersize',10)
    title('Pozitie Y');
    grid on;
    axis tight
    ax2.Position = ax2.Position + [0 0 0.05 0.05];
    
    if (filter_type == 'L' || filter_type == 'l')
        ax3 = subplot(4,2,3);
        plot(vehicle_velocity_history_x / 100 ,'color',[0 0.4470 0.7410],'linew',1.5)
        hold on
        plot(chaser_velocity_history_x / 100,'color',[0.9290 0.6940 0.1250],'linew',1.5)
        plot((estimated_state_history_x_vel) / 100 ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        title('Viteza X');
        grid on;
        axis tight
        ax3.Position = ax3.Position + [0 -0.05 0.05 0.05];
        ax4 = subplot(4,2,4);
        plot((vehicle_velocity_history_y / 100) ,'color',[0 0.4470 0.7410],'linew',1.5)
        hold on
        plot(chaser_velocity_history_y / 100,'color',[0.9290 0.6940 0.1250],'linew',1.5)
        plot((estimated_state_history_y_vel) / 100 ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        title('Viteza Y')
        grid on;
        axis tight
        ax4.Position = ax4.Position + [0 -0.05 0.05 0.05];
    end
    
    if (filter_type == 'E' || filter_type == 'e')  || (filter_type == 'U' || filter_type == 'u')
        ax3 = subplot(4,2,3);
        plot(wrapTo180(rad2deg(vehicle_heading_history)),'color',[0 0.4470 0.7410],'linew',1.5)
        hold on
        plot(wrapTo180(rad2deg(estimated_state_history_yaw)) ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        title('Orientare');
        grid on;
        axis tight
        ax3.Position = ax3.Position + [0 -0.05 0.05 0.05];
        ax4 = subplot(4,2,4);
        plot(vehicle_velocity_history / 100 ,'color',[0 0.4470 0.7410],'linew',1.5)
        hold on
        plot((estimated_state_history_vel) / 100 ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        title('Viteza')
        grid on;
        axis tight
        ax4.Position = ax4.Position + [0 -0.05 0.05 0.05];
    end
    
    ax5 = subplot(4,2,5);
    plot(estimation_error_history(1:4:end) / 100 ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
    hold on
    plot(chaser_error_history(1:4:end) / 100 ,'color',[0.9290 0.6940 0.1250],'linew',1.5)
    plot([0, length(estimation_error_history(1:4:end))],[0,0],'r--','linew',1.5)
    plot(3 * sqrt(estimated_covariance_history_x_pos) ,'color',[0.890000 0.310000 0.610000],'linew',1.5)
    plot(-3 * sqrt(estimated_covariance_history_x_pos),'color',[0.890000 0.310000 0.610000],'linew',1.5)
    ax5.Position = ax5.Position + [0 -0.06 0.05 0.05];
    axis tight
    grid on; title('Eroare Pozitie X')
    ax6 = subplot(4,2,6);
    plot(estimation_error_history(2:4:end) / 100,'color',[0.4660 0.6740 0.1880],'linew',1.5)
    hold on
    plot(chaser_error_history(2:4:end) / 100 ,'color',[0.9290 0.6940 0.1250],'linew',1.5)
    plot([0, length(estimation_error_history(2:4:end))],[0,0],'r--','linew',1.5)
    plot(3 * sqrt(estimated_covariance_history_y_pos),'color',[0.890000 0.310000 0.610000],'linew',1.5)
    plot(-3 * sqrt(estimated_covariance_history_y_pos),'color',[0.890000 0.310000 0.610000],'linew',1.5)
    ax6.Position = ax6.Position + [0 -0.06 0.05 0.05];
    axis tight
    grid on; title('Eroare Pozitie Y');
    
    if (filter_type == 'L' || filter_type == 'l')
        ax7 = subplot(4,2,7);
        plot((estimation_error_history(3:4:end)) / 100 ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        hold on
        plot(chaser_error_history(3:4:end) / 100 ,'color',[0.9290 0.6940 0.1250],'linew',1.5)
        plot([0, length(estimation_error_history(3:4:end))],[0,0],'r--','linew',1.5)
        plot(3 * sqrt(estimated_covariance_history_x_vel) ,'color',[0.890000 0.310000 0.610000],'linew',1.5)
        plot(-3 * sqrt(estimated_covariance_history_x_vel) ,'color',[0.890000 0.310000 0.610000],'linew',1.5)
        ax7.Position = ax7.Position + [0 -0.09 0.05 0.05];
        axis tight
        grid on; title('Eroare Viteza X');
        ax8 = subplot(4,2,8);
        plot((estimation_error_history(4:4:end)) / 100,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        hold on
        plot(chaser_error_history(4:4:end) / 100 ,'color',[0.9290 0.6940 0.1250],'linew',1.5)
        plot([0, length(estimation_error_history(4:4:end))],[0,0],'r--','linew',1.5)
        plot(3 * sqrt(estimated_covariance_history_y_vel),'color',[0.890000 0.310000 0.610000],'linew',1.5)
        plot(-3 * sqrt(estimated_covariance_history_y_vel),'color',[0.890000 0.310000 0.610000],'linew',1.5)
        ax8.Position = ax8.Position + [0 -0.09 0.05 0.05];
        axis tight
        grid on; title('Eroare Viteza Y');
    end
    
    if (filter_type == 'E' || filter_type == 'e') || (filter_type == 'U' || filter_type == 'u')
        ax7 = subplot(4,2,7);
        plot((estimation_error_history(3:4:end)) ,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        hold on
        plot([0, length(estimation_error_history(3:4:end))],[0,0],'r--','linew',1.5)
        plot(3 * rad2deg(sqrt((estimated_covariance_history_yaw))) ,'color',[0.890000 0.310000 0.610000],'linew',1.5)
        plot(-3 * rad2deg(sqrt((estimated_covariance_history_yaw))) ,'color',[0.890000 0.310000 0.610000],'linew',1.5)
        ax7.Position = ax7.Position + [0 -0.09 0.05 0.05];
        axis tight
        grid on; title('Eroare Orientare');
        ax8 = subplot(4,2,8);
        plot((estimation_error_history(4:4:end)) / 100,'color',[0.4660 0.6740 0.1880],'linew',1.5)
        hold on
        plot([0, length(estimation_error_history(3:4:end))],[0,0],'r--','linew',1.5)
        plot(3 * sqrt(estimated_covariance_history_vel),'color',[0.890000 0.310000 0.610000],'linew',1.5)
        plot(-3 * sqrt(estimated_covariance_history_vel),'color',[0.890000 0.310000 0.610000],'linew',1.5)
        ax8.Position = ax8.Position + [0 -0.09 0.05 0.05];
        axis tight
        grid on; title('Eroare Viteza');
    end
    
    legend(ax1,'reala','controlata','estimata','masurata','location','westoutside','fontsize',14)
    
    if (filter_type == 'L' || filter_type == 'l')
        legend(ax8,'estimare','reglare','nul (0)','varianta','location','eastoutside','fontsize',14)
    else
        legend(ax8,'estimare','nul (0)','varianta','location','eastoutside','fontsize',14)
    end
end
