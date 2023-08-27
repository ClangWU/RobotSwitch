%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%% Import data
ahrs_data = dlmread('ahrs_data.txt');
ahrs.Time     = ahrs_data(:, 1);
ahrs.quaternion = ahrs_data(:, 2:5);
ahrs.gyr      = ahrs_data(:, 6:8);
ahrs.acc    = ahrs_data(:, 9:11);

samplePeriod = 1/200;
% Plot
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(ahrs.gyr(:,1), 'r');
plot(ahrs.gyr(:,2), 'g');
plot(ahrs.gyr(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(ahrs.acc(:,1), 'r');
plot(ahrs.acc(:,2), 'g');
plot(ahrs.acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(ahrs.gyr));     % rotation matrix describing sensor relative to Earth

%ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(ahrs.gyr)
    R(:,:,i) = quatern2rotMat(ahrs.quaternion(i,:))';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(ahrs.acc));  % accelerometer in Earth frame

for i = 1:length(ahrs.acc)
    tcAcc(i,:) = R(:,:,i) * ahrs.acc(i,:)';
end

%%clang
gravity_estimate = mean(tcAcc(:, 3));

% Plot
figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1) * gravity_estimate];

order = 2;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linAccHP = filtfilt(b, a, linAcc);

figure('NumberTitle', 'off', 'Name', 'Linear Acceleration Analysis');
subplot(2,1,1);
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

subplot(2,1,2);
hold on;
plot(linAccHP(:,1), 'r');
plot(linAccHP(:,2), 'g');
plot(linAccHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration HP');
legend('X', 'Y', 'Z');
%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAccHP));
linVel_nxp = zeros(size(linAccHP));
for i = 2:length(linAccHP)
    linVel(i,:) = linVel(i-1,:) + linAccHP(i,:) * samplePeriod;
    linVel_nxp(i,:) = linVel_nxp(i-1,:) + linAccHP(i-1,:) * samplePeriod + (linAccHP(i,:) + linAccHP(i-1,:)) * 0.5 * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
subplot(2,1,1);
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');

subplot(2,1,2);
hold on;
plot(linVel_nxp(:,1), 'r');
plot(linVel_nxp(:,2), 'g');
plot(linVel_nxp(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity NXP');
legend('X', 'Y', 'Z');
%% Position reintegration
linPos_ = zeros(size(linVel_nxp));
linPos_nxp = zeros(size(linVel_nxp));

for i = 2:length(linVel_nxp)
    linPos_(i,:) = linPos_(i-1,:) + linVel(i,:) *  samplePeriod; 
    linPos_nxp(i,:) = linPos_nxp(i-1,:) + linVel_nxp(i-1,:) * samplePeriod + (linVel_nxp(i,:) + linVel_nxp(i-1,:)) * 0.5 * samplePeriod; 
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Position');
subplot(2,1,1);
hold on;
plot(linPos_(:,1), 'r');
plot(linPos_(:,2), 'g');
plot(linPos_(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position ');
legend('X', 'Y', 'Z');

subplot(2,1,2);
hold on;
plot(linPos_nxp(:,1), 'r');
plot(linPos_nxp(:,2), 'g');
plot(linPos_nxp(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position a');
legend('X', 'Y', 'Z');

%% Play animation

SamplePlotFreq = 8;
%%linPosHP_process
% First animation
SixDOFanimation(linPos_nxp, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.01, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));  

%% End of script