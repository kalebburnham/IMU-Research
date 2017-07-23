close all;
clc;
clear;

load('logfile_web_straight.mat');
addpath('../quaternion_library');
addpath('..');

period = 1/fs;
gravity = [0 0 9.786];

sampleSize = 5000;

beta = 0;
pitchCorrection = 0.5790; % radians
rollCorrection = 0.0875;
yawCorrection = 0;

GravityAcc = zeros(size(Acc));
for t = 1:length(Acc)
    GravityAcc(t,:) = [0 0 9.786] * DCM(:,:,t);
end

% Find initial quaternion and create AHRS
InitialQuaternion = rotMat2quatern(DCM(:,:,1));
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', InitialQuaternion);

quat_est = zeros(length(Acc), 4);
euler_est = zeros(size(Acc));
DCM_est = zeros(size(DCM));

% Find acceleration in global frame
for t = 1:length(Acc)
    % Update quaternion
    AHRS.Update(Gyr(t,:), GravityAcc(t,:), Mag(t,:));
    quat_est(t,:) = AHRS.Quaternion;
    
    % "use conjugate for sensor frame relative to Earth" - Madgwick
    quat_est(t,:) = quaternConj(quat_est(t,:));
    
    euler_est(t,:) = quatern2euler(quat_est(t,:));
end

% Rotate real acceleration into global frame
for t = 1:length(Acc)
    Acc(t,:) = quatrotate(quat_est(t,:), Acc(t,:)) - gravity;
end

vel_est = zeros(size(Acc));
pos_est = zeros(size(Acc));
for t = 1000:length(Acc)
    vel_est(t,:) = vel_est(t-1,:) + Acc(t,:) * period;
    pos_est(t,:) = pos_est(t-1,:) + vel_est(t,:) * period;
end

figure;
hold on;
plot(1:sampleSize, euler_est(1:sampleSize,1), 'r');
plot(1:sampleSize, Euler(1:sampleSize,1), 'g');
hold off;

figure;
hold on;
plot(1:sampleSize, euler_est(1:sampleSize,2), 'r');
plot(1:sampleSize, Euler(1:sampleSize,2), 'g');
hold off;

figure;
hold on;
plot(1:sampleSize, euler_est(1:sampleSize,3), 'r');
plot(1:sampleSize, Euler(1:sampleSize,3), 'g');
hold off;

% 2D results
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, pos_est(1:sampleSize,1), 'r');
plot(1:sampleSize, Pos(1:sampleSize,1), 'g');
title('Position Estimate on X-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, pos_est(1:sampleSize,2), 'r');
plot(1:sampleSize, Pos(1:sampleSize,2), 'g');
title('Position Estimate on Y-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, pos_est(1:sampleSize,3), 'r');
plot(1:sampleSize, Pos(1:sampleSize,3), 'g');
title('Position Estimate on Z-axis');
legend('Estimate', 'Ground Truth');
hold off;