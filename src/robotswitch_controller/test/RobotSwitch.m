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

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1) * 9.89];

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc1(:,1), 'r');
plot(linAcc1(:,2), 'g');
plot(linAcc1(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration + gravity_estimate');
legend('X', 'Y', 'Z');

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
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
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
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

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% Calculate linear position (integrate velocity)

linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear position');
legend('X', 'Y', 'Z');

%% High-pass filter linear position to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');
plot(linPosHP(:,2), 'g');
plot(linPosHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% Play animation

SamplePlotFreq = 8;

SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script