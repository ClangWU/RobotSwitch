%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%% Import data
ahrs_static_data = dlmread('ahrs_static.txt');
ahrs.Time    = ahrs_static_data(:, 1);
ahrs.rawacc  = ahrs_static_data(:, 2:4);
ahrs.realacc = ahrs_static_data(:, 5:7);
ahrs.pos     = ahrs_static_data(:, 8:10);
ahrs.quaternion  = ahrs_static_data(:, 11:14);

R = zeros(3,3,length(ahrs.rawacc));     % rotation matrix describing sensor relative to Earth
for i = 1:length(ahrs.rawacc)
    R(:,:,i) = quatern2rotMat(ahrs.quaternion(i,:))';    % transpose because ahrs provides Earth relative to sensor
end
tcAcc = zeros(size(ahrs.rawacc));  % accelerometer in Earth frame
for i = 1:length(ahrs.rawacc)
    tcAcc(i,:) = R(:,:,i) * ahrs.rawacc(i,:)';
end
linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1) * 9.89062];
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration Analysis');
subplot(2,1,1);
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration matlab');
legend('X', 'Y', 'Z');

subplot(2,1,2);
hold on;
plot(ahrs.realacc(:,1), 'r');
plot(ahrs.realacc(:,2), 'g');
plot(ahrs.realacc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration CPP');
legend('X', 'Y', 'Z');
