%% Import and Process data
ahrs_data = dlmread('ahrs_data.txt');
ahrs.Time     = ahrs_data(:, 1);
ahrs.quaternion = ahrs_data(:, 2:5);
ahrs.gyr      = ahrs_data(:, 6:8);
ahrs.acc    = ahrs_data(:, 9:11);

R = zeros(3,3,length(ahrs.gyr));     % rotation matrix describing sensor relative to Earth

%ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(ahrs.gyr)
    R(:,:,i) = quatern2rotMat(ahrs.quaternion(i,:))';    % transpose because ahrs provides Earth relative to sensor
end

tcAcc = zeros(size(ahrs.acc));  % accelerometer in Earth frame
for i = 1:length(ahrs.acc)
    tcAcc(i,:) = R(:,:,i) * ahrs.acc(i,:)';
end
gravity_estimate = mean(tcAcc(:, 3));
linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1) * gravity_estimate];

%% FFT and plot for acceleration data
samplePeriod = ahrs.Time(2) - ahrs.Time(1); % Assuming uniform sampling
Fs = 1/samplePeriod; % Sampling frequency
L = length(ahrs.Time); % Length of signal
f = Fs*(0:(L/2))/L; % Frequency vector

figure;

% For X-direction
accX = linAcc(:,1);
Y_X = fft(accX);
P2_X = abs(Y_X/L);
P1_X = P2_X(1:L/2+1);
P1_X(2:end-1) = 2*P1_X(2:end-1);
subplot(3,1,1);
plot(f, P1_X);
title('(X-axis)')
xlabel('Hz')
ylabel('|Amplitude|')

% For Y-direction
accY = linAcc(:,2);
Y_Y = fft(accY);
P2_Y = abs(Y_Y/L);
P1_Y = P2_Y(1:L/2+1);
P1_Y(2:end-1) = 2*P1_Y(2:end-1);
subplot(3,1,2);
plot(f, P1_Y);
title('(Y-axis)')
xlabel('Hz')
ylabel('|Amplitude|')

% For Z-direction
accZ = linAcc(:,3);
Y_Z = fft(accZ);
P2_Z = abs(Y_Z/L);
P1_Z = P2_Z(1:L/2+1);
P1_Z(2:end-1) = 2*P1_Z(2:end-1);
subplot(3,1,3);
plot(f, P1_Z);
title('(Z-axis)')
xlabel('Hz')
ylabel('|Amplitude|')

% Adjust layout
sgtitle('¼ÓËÙ¶ÈÆµÆ×')
tight_layout();