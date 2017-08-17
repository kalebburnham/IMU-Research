%% Config
close all;
clear;
...clc;

load('../data/logfile_web_straight.mat');
addpath('../quaternion_library');
addpath('..');

period = 1/fs;
sampleSize = length(Acc);
start = 1;

%% Variable Declaration
DCM_est1 = zeros(3,3,sampleSize); DCM_est2 = zeros(3,3,sampleSize);
quat_est1 = zeros(sampleSize,4);  quat_est2 = zeros(sampleSize,4);
acc_est1 = zeros(sampleSize,3);   acc_est2 = zeros(sampleSize,3);
euler_est1 = zeros(sampleSize,3); euler_est2 = zeros(sampleSize,3);
vel_est1 = zeros(sampleSize,3);   vel_est2 = zeros(sampleSize,3);
pos_est1 = zeros(sampleSize,3);   pos_est2 = zeros(sampleSize,3);

%% Settings
rollCorrection = 5*pi/180;
pitchCorrection = 15*pi/180;
yawCorrection = 0;

% found by DCM(:,:,1) * Rx * Rz_
%rollCorrection2 = 165*pi/180;  |
%pitchCorrection2 = 5*pi/180;   |___\  These values, though close, lead to
%yawCorrection2 = -90*pi/180;  _|   /  extremely inaccurate pos_est.
rollCorrection2 = 164.9453*pi/180;
pitchCorrection2 = 4.8292*pi/180;
yawCorrection2 = -91.2972*pi/180;

theta = pi; % Rotation around x axis 180 degrees
Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
theta = pi/2; % Rotation around z axis 90 degrees
Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

% Ground truth DCM for the new IMU
DCM2 = zeros(size(DCM));
for t = 1:sampleSize
    DCM2(:,:,t) = DCM(:,:,t) * Rx * Rz;
end

Acc2 = Acc * Rx * Rz;
Gyr2 = Gyr * Rx * Rz;
Mag2 = Mag * Rx * Rz;

[Acc,Gyr] = addError(Acc,Gyr);
[Acc2,Gyr2] = addError(Acc2,Gyr2);

%% Function
rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm');
rotm2 = euler2rotMat(rollCorrection2, pitchCorrection2, yawCorrection2);
InitialQuaternion2 = rotMat2quatern(rotm2');

gamma = 0;
AHRS1 = MadgwickAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion, 'Beta', gamma);
AHRS2 = MadgwickAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion2, 'Beta', gamma);

for t = 1:sampleSize
    AHRS1.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    AHRS2.Update(Gyr2(t,:), Acc2(t,:), Mag2(t,:));
    quat_est1(t,:) = AHRS1.Quaternion;
    quat_est2(t,:) = AHRS2.Quaternion;

    DCM_est1(:,:,t) = quatern2rotMat(quat_est1(t,:))';  % always invert rotation matrix when using rotMat2quatern
    DCM_est2(:,:,t) = quatern2rotMat(quat_est2(t,:))';
    
    acc_est1(t,:) = Acc(t,:) * inv(DCM_est1(:,:,t));
    acc_est2(t,:) = Acc2(t,:) * inv(DCM_est2(:,:,t));
    
    acc_est1(t,3) = acc_est1(t,3) - gravity;
    acc_est2(t,3) = acc_est2(t,3) - gravity;
end

for t = 2:sampleSize
    vel_est1(t,:) = vel_est1(t-1,:) + (acc_est1(t,:) * period); % Acc -> vel
    pos_est1(t,:) = pos_est1(t-1,:) + (vel_est1(t,:) * period); % vel -> pos
    vel_est2(t,:) = vel_est2(t-1,:) + (acc_est2(t,:) * period); % Acc -> vel
    pos_est2(t,:) = pos_est2(t-1,:) + (vel_est2(t,:) * period); % vel -> pos
end

avg_pos_est = (pos_est1 + pos_est2) / 2;
RMSE_pos_est1 = sqrt(mean((Pos - pos_est1) .^2 ))
RMSE_pos_est2 = sqrt(mean((Pos - pos_est2) .^ 2))
RMSE_avg_pos_est = sqrt(mean((Pos - avg_pos_est) .^ 2))


figure;
axis(1) = subplot(3,1,1);
hold on;
plot(start:sampleSize, acc_est1(1:sampleSize,1), 'r');
plot(start:sampleSize, acc_est2(1:sampleSize,1), 'b');
title('Accelerations');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(start:sampleSize, acc_est1(1:sampleSize,2), 'r');
plot(start:sampleSize, acc_est2(1:sampleSize,2), 'b');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(start:sampleSize, acc_est1(1:sampleSize,3), 'r');
plot(start:sampleSize, acc_est2(1:sampleSize,3), 'b');
legend('Original', 'New');
hold off;
set(gcf,'Color',[1 1 1])

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(start:sampleSize, pos_est1(start:sampleSize,1), 'r');
plot(start:sampleSize, pos_est2(start:sampleSize,1), 'b');
plot(start:sampleSize, Pos(start:sampleSize,1), 'g');
plot(start:sampleSize, avg_pos_est(start:sampleSize,1), 'c');
title('Position Estimate');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(start:sampleSize, pos_est1(start:sampleSize,2), 'r');
plot(start:sampleSize, pos_est2(start:sampleSize,2), 'b');
plot(start:sampleSize, Pos(start:sampleSize,2), 'g');
plot(start:sampleSize, avg_pos_est(start:sampleSize,2), 'c');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(start:sampleSize, pos_est1(start:sampleSize,3), 'r');
plot(start:sampleSize, pos_est2(start:sampleSize,3), 'b');
plot(start:sampleSize, Pos(start:sampleSize,3), 'g');
plot(start:sampleSize, avg_pos_est(start:sampleSize,3), 'c');
legend('Original', 'New', 'Ground Truth', 'Average');
hold off;
set(gcf,'Color',[1 1 1])