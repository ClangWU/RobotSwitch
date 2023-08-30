%% Housekeeping
 
addpath('cpp_filter');	% include x-IMU MATLAB library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

%% Import data
ahrs_static_data = dlmread('ahrs_dyn_hpf.txt');
ahrs.Time    = ahrs_static_data(:, 1);
ahrs.acc  = ahrs_static_data(:, 2:4);
ahrs.realacc = ahrs_static_data(:, 5:7);
ahrs.vel     = ahrs_static_data(:, 8:10);
ahrs.pos     = ahrs_static_data(:, 11:13);
ahrs.quaternion  = ahrs_static_data(:, 14:17);

% 初始化
dt = 0.005;
vel_filter = CppHighPassFilter();
pos_filter = CppHighPassFilter();
vel_filter.InitFilter(1, 1/dt, 0.1); % 示例值: 1阶, 0.1 Hz截止
pos_filter.InitFilter(1, 1/dt, 0.1); % 示例值: 1阶, 0.1 Hz截止

% 提前定义输出的大小，以便于预分配空间并提高性能
filtered_vel = zeros(size(ahrs.realacc));
filtered_pos = zeros(size(ahrs.realacc));

% 临时变量
real_vel = zeros(1, 3);
real_pos = zeros(1, 3);

for i = 1:length(ahrs.Time)
    real_acc = ahrs.realacc(i, :);
    
    real_vel = real_vel + real_acc * dt;
    vel_ft = vel_filter.process(real_vel);
    
    real_pos = real_pos + vel_ft * dt;
    pos_ft = pos_filter.process(real_pos);
    
    real_vel = vel_ft;
    real_pos = pos_ft;

    % 存储过滤后的值
    filtered_vel(i, :) = vel_ft;
    filtered_pos(i, :) = pos_ft;
end

% 绘制 filtered_vel 
figure;
plot(ahrs.Time, filtered_vel(:, 1), 'r', 'DisplayName', 'Vel x-axis');
hold on;
plot(ahrs.Time, filtered_vel(:, 2), 'g', 'DisplayName', 'Vel y-axis');
plot(ahrs.Time, filtered_vel(:, 3), 'b', 'DisplayName', 'Vel z-axis');
xlabel('Time (s)');
ylabel('Velocity');
title('Filtered Velocities over Time');
legend;
grid on;

% 绘制 filtered_pos 
figure;
plot(ahrs.Time, filtered_pos(:, 1), 'r', 'DisplayName', 'Pos x-axis');
hold on;
plot(ahrs.Time, filtered_pos(:, 2), 'g', 'DisplayName', 'Pos y-axis');
plot(ahrs.Time, filtered_pos(:, 3), 'b', 'DisplayName', 'Pos z-axis');
xlabel('Time (s)');
ylabel('Position');
title('Filtered Positions over Time');
legend;
grid on;

