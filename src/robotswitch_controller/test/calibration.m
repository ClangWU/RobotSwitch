%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%% Import data
ahrs_static_data = dlmread('ahrs_static.txt');
ahrs.Time    = ahrs_static_data(:, 1);
ahrs.acc  = ahrs_static_data(:, 2:4);
ahrs.realacc = ahrs_static_data(:, 5:7);
ahrs.pos     = ahrs_static_data(:, 8:10);
ahrs.quaternion  = ahrs_static_data(:, 11:14);

samplePeriod = 0.005; % Assuming uniform sampling
figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(ahrs.realacc(:,1), 'r');
plot(ahrs.realacc(:,2), 'g');
plot(ahrs.realacc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(ahrs.acc));     % rotation matrix describing sensor relative to Earth

%ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(ahrs.acc)
    R(:,:,i) = quatern2rotMat(ahrs.quaternion(i,:))';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(ahrs.acc));  % accelerometer in Earth frame

for i = 1:length(ahrs.acc)
    tcAcc(i,:) = R(:,:,i) * ahrs.acc(i,:)';
end

%%clang
gravity_estimate = mean(tcAcc(:, 3));

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = ahrs.realacc;
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration Analysis');
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod ;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');

%% High-pass filter linear velocity to remove drift

order = 2;
filtCutOff = 0.1;
N = 200; % 假设的滤波器阶数，可以根据需要调整
fCutNormalized = 2*filtCutOff*samplePeriod; % 归一化截止频率

% 设计FIR高通滤波器
b_fir = fir1(N, fCutNormalized, 'high');
a_fir = 1; % 对于FIR滤波器，a始终为1

% 使用filter函数进行实时滤波
linVel_filtfilt = filter(b_fir, a_fir, linVel);

[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
Hd = dsp.IIRFilter('Numerator', b, 'Denominator', a);
%linVel_filtfilt = step(Hd, linVel);
% linVel_filtfilt = filtfilt(b, a, linVel);
%linVel_filtfilt = filter(b, a, linVel);


% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVel_filtfilt(:,1), 'r');
plot(linVel_filtfilt(:,2), 'g');
plot(linVel_filtfilt(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity after');
legend('X', 'Y', 'Z');

%% Calculate linear position (integrate velocity)

linPos_ = zeros(size(linVel_filtfilt));
for i = 2:length(linVel_filtfilt)
    linPos_(i,:) = linPos_(i-1,:) + linVel_filtfilt(i,:) *  samplePeriod; 
end
order = 2;
filtCutOff = 0.1;
N = 200; % 假设的滤波器阶数，可以根据需要调整
fCutNormalized = 2*filtCutOff*samplePeriod; % 归一化截止频率

% 设计FIR高通滤波器
b_fir = fir1(N, fCutNormalized, 'high');
a_fir = 1; % 对于FIR滤波器，a始终为1
linPos_filtfilt = filter(b_fir, a_fir, linPos_);
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
Hd = dsp.IIRFilter('Numerator', b, 'Denominator', a);
%linPos_filtfilt = filter(b, a, linPos_);
% linPos_filtfilt = filtfilt(b, a, linPos_);
% linPos_filtfilt = step(Hd, linPos_);
% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Position');
subplot(2,1,1);
hold on;
plot(linPos_(:,1), 'r');
plot(linPos_(:,2), 'g');
plot(linPos_(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position');
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


L = length(ahrs.Time); % Length of signal
Fs = 1/samplePeriod; % Sampling frequency
f = Fs*(0:(L/2))/L; % Frequency vector
figure;
% For X-direction
Y_X = fft(linPos_filtfilt(:,1));
P2_X = abs(Y_X/L);
P1_X = P2_X(1:L/2+1);
P1_X(2:end-1) = 2*P1_X(2:end-1);
subplot(3,1,1);
plot(f, P1_X);
title('(X-axis)')
xlabel('Hz')
ylabel('|Amplitude|')

% For Y-direction
Y_Y = fft(linPos_filtfilt(:,2));
P2_Y = abs(Y_Y/L);
P1_Y = P2_Y(1:L/2+1);
P1_Y(2:end-1) = 2*P1_Y(2:end-1);
subplot(3,1,2);
plot(f, P1_Y);
title('(Y-axis)')
xlabel('Hz')
ylabel('|Amplitude|')

% For Z-direction
Y_Z = fft(linPos_filtfilt(:,3));
P2_Z = abs(Y_Z/L);
P1_Z = P2_Z(1:L/2+1);
P1_Z(2:end-1) = 2*P1_Z(2:end-1);
subplot(3,1,3);
plot(f, P1_Z);
title('(Z-axis)')
xlabel('Hz')
ylabel('|Amplitude|')

% Adjust layout
sgtitle('位置滤波后频谱')

%% Play animation

% SamplePlotFreq = 8;
% %%linPosHP_process
% % First animation
% SixDOFanimation(linPos_filtfilt, R, ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
%                 'Position', [9 39 1280 720], ...
%                 'AxisLength', 0.01, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
%                 'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));  
    
%% End of script