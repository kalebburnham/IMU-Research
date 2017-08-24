% Simulates an IMU located at [1 0 0] relative to the first IMU with no
% change in starting orientation.

%% Config
close all;
clear;
...clc;

load('../data/logfile_web_straight.mat');
addpath('../quaternion_library');
addpath('..');

period = 1/fs;
sampleSize = 1050;
start = 1;

%% Variable Declaration
DCM_est = zeros(3,3,sampleSize); DCM_est2 = zeros(3,3,sampleSize);
quat_est = zeros(sampleSize,4);  quat_est2 = zeros(sampleSize,4);
acc_est = zeros(sampleSize,3);   acc_est2 = zeros(sampleSize,3);
euler_est = zeros(sampleSize,3); euler_est2 = zeros(sampleSize,3);
vel_est = zeros(sampleSize,3);   vel_est2 = zeros(sampleSize,3);
pos_est = zeros(sampleSize,3);   pos_est2 = zeros(sampleSize,3);
AccNoGrav = zeros(size(Acc));
OffsetAcc = zeros(sampleSize,3);
GyrAccel = zeros(sampleSize,3);


%% Settings
rollCorrection = 5*pi/180;
pitchCorrection = 15*pi/180;
yawCorrection = 0;

%% Function

% (1) Calculate Gyr Acceleration
for t = 2:sampleSize
    GyrAccel(t,:) = Gyr(t,:) - Gyr(t-1,:);
end

% (2) Add GyrAccel to Acc
OffsetAcc(:,1) = Acc(1:sampleSize,1);
OffsetAcc(:,2) = Acc(1:sampleSize,2) + GyrAccel(:,3);
OffsetAcc(:,3) = Acc(1:sampleSize,3) - GyrAccel(:,2);


% (3) Add Noise
...[OffsetAcc, Gyr(1:sampleSize,:)] = addError(OffsetAcc, Gyr(1:sampleSize,:));
    
% (4) Remove Gravity
OffsetAccNoGrav = zeros(size(OffsetAcc));
for t = 1:sampleSize
    OffsetAccNoGrav(t,:) = (DCM(:,:,t) * OffsetAcc(t,:)')';
    OffsetAccNoGrav(t,:) = OffsetAccNoGrav(t,:) - [0 0 9.8];
end

% (6) Remove GyrAccel from OffsetAcc
OffsetAcc(:,2) = OffsetAcc(:,2) - GyrAccel(:,3);
OffsetAcc(:,3) = OffsetAcc(:,3) + GyrAccel(:,2);



for t = 2:sampleSize
    vel_est(t,:) = vel_est(t-1,:) + (Acc(t,:) * period); % Acc -> vel
    pos_est(t,:) = pos_est(t-1,:) + (vel_est(t,:) * period); % vel -> pos
end


%% Graphs
figure;
hold on;
plot(1:sampleSize, Acc(1:sampleSize,1), 'r');
plot(1:sampleSize, Acc(1:sampleSize,2), 'g');
plot(1:sampleSize, Acc(1:sampleSize,3), 'b');
title('Original Acceleration');
hold off;

figure;
hold on;
plot(1:sampleSize, Gyr(1:sampleSize,1), 'r');
plot(1:sampleSize, Gyr(1:sampleSize,2), 'g');
plot(1:sampleSize, Gyr(1:sampleSize,3), 'b');
title('Gyro');
hold off;

figure;
hold on;
plot(1:sampleSize, OffsetAcc(1:sampleSize,1), 'r');
plot(1:sampleSize, OffsetAcc(1:sampleSize,2), 'g');
plot(1:sampleSize, OffsetAcc(1:sampleSize,3), 'b');
title('Offset Acc');
hold off;