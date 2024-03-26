%% Housekeeping
 
addpath('cpp_filter');	% include x-IMU MATLAB library
addpath('cpp_filter');	% include x-IMU MATLAB library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

%% Import data
ahrs_static_data = dlmread('ahrs_dyn_hpf2_1.0.txt');
ahrs.Time    = ahrs_static_data(:, 1);
ahrs.acc  = ahrs_static_data(:, 2:4);
ahrs.realacc = ahrs_static_data(:, 5:7);
ahrs.vel     = ahrs_static_data(:, 8:10);
ahrs.pos     = ahrs_static_data(:, 11:13);
ahrs.quaternion  = ahrs_static_data(:, 14:17);

% figure('NumberTitle', 'off', 'Name', 'Linear acc');
% % 绘制 filtered_pos 
% hold on;
% plot(ahrs.Time, ahrs.realacc(:, 1), 'r', 'DisplayName', 'Pos x-axis');
% plot(ahrs.Time, ahrs.realacc(:, 2), 'g', 'DisplayName', 'Pos y-axis');
% plot(ahrs.Time, ahrs.realacc(:, 3), 'b', 'DisplayName', 'Pos z-axis');
% xlabel('sample');
% ylabel('m');
% title(' acc over Time');
% legend('X', 'Y', 'Z');
% order = 2;
% filtCutOff = 20;
% [b, a] = butter(order, (2*filtCutOff)/200, 'low');
% ahrs.realacc = filtfilt(b, a, ahrs.realacc);
% figure('NumberTitle', 'off', 'Name', 'Linear acc lp');
% % 绘制 filtered_pos 
% hold on;
% plot(ahrs.Time, ahrs.realacc(:, 1), 'r', 'DisplayName', 'Pos x-axis');
% plot(ahrs.Time, ahrs.realacc(:, 2), 'g', 'DisplayName', 'Pos y-axis');
% plot(ahrs.Time, ahrs.realacc(:, 3), 'b', 'DisplayName', 'Pos z-axis');
% xlabel('sample');
% ylabel('m');
% title('Filtered acc over Time');
% legend('X', 'Y', 'Z');

% 初始化
dt = 0.005;
vel_filter = FIRFilter(2, 200, 1);
pos_filter = FIRFilter(2, 200, 1);
% vel_filter.matlabInitFilter(1,  0.002); % 示例值: 1阶, 0.1 Hz截止
% pos_filter.matlabInitFilter(2,  0.001); % 示例值: 1阶, 0.1 Hz截止

% [b_butter, a_butter] = butter(1, (2*0.1)/(200), 'high');

% 获取你自定义滤波器的系数
% b_custom = vel_filter.getbCoeffs;  % 以2阶为例
% a_custom = vel_filter.getaCoeffs;

% 计算频率响应
% [H_butter, w] = freqz(b_butter, a_butter);
% [H_custom, ~] = freqz(b_custom, a_custom, w);
% 
% % 绘制频率响应
% figure;
% plot(w, abs(H_butter), 'r', w, abs(H_custom), 'b--');
% legend('Butter', 'Custom');
% title('Frequency Response');
% 
% a = vel_filter.getDenominatorCoeffs();
% poles = roots(a);
% 
% if all(abs(poles) < 1)
%     disp('The filter is stable');
% else
%     disp('The filter is unstable');
% end

real_vel = [0; 0; 0]; % Initialize as a column vector
real_pos = [0; 0; 0]; % Initialize as a column vector

filtered_vel = zeros(length(ahrs.Time), 3);
filtered_pos = zeros(length(ahrs.Time), 3);

for i = 1:length(ahrs.Time)
    real_acc = ahrs.realacc(i, :)';
    
    real_vel = real_vel + real_acc * dt;
    vel_ft = vel_filter.process(real_vel);
%     real_pos = real_pos + real_vel * dt;
    real_pos = real_pos + reshape(vel_ft, [3,1]) * dt; % Ensure vel_ft is a column vector
    pos_ft = pos_filter.process(real_pos);
    
    real_vel = vel_ft; % Ensure vel_ft is a column vector
    real_pos = pos_ft;

    % 存储过滤后的值
    filtered_vel(i, :) = vel_ft';
    filtered_pos(i, :) = pos_ft';
%     filtered_pos(i, :) = real_pos';
%     filtered_vel(i, :) = real_vel';
end

linAcc = ahrs.realacc;
linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * 0.005 ;
end
order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(200), 'high');
linVel_filtfilt = filtfilt(b, a, linVel);
linPos_ = zeros(size(linVel_filtfilt));
for i = 2:length(linVel_filtfilt)
    linPos_(i,:) = linPos_(i-1,:) + linVel_filtfilt(i,:) *  0.005; 
end
order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(200), 'high');
linPos_filtfilt = filtfilt(b, a, linPos_);

figure('NumberTitle', 'off', 'Name', 'Linear Position');
% 绘制 filtered_pos 
subplot(2,1,1);
hold on;
plot(ahrs.Time, filtered_pos(:, 1), 'r', 'DisplayName', 'Pos x-axis');
plot(ahrs.Time, filtered_pos(:, 2), 'g', 'DisplayName', 'Pos y-axis');
plot(ahrs.Time, filtered_pos(:, 3), 'b', 'DisplayName', 'Pos z-axis');
xlabel('sample');
ylabel('m');
title('Filtered Positions over Time');
legend('X', 'Y', 'Z');

subplot(2,1,2);
hold on;
plot(linPos_filtfilt(:,1), 'r');
plot(linPos_filtfilt(:,2), 'g');
plot(linPos_filtfilt(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position HP');
legend('X', 'Y', 'Z');
