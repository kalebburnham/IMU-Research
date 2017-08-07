close all;
clear;
clc;

load('straight walk, 1000 steps.mat');
addpath('quaternion_library');

period = 1/fs;
sampleSize = 10000;
beta = .5;

pitchCorrection = 30*pi/180; % radians
rollCorrection = 5*pi/180;
yawCorrection = 0;

rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm);
AHRS = BasicAHRS('SamplePeriod', 1/100, 'Quaternion', InitialQuaternion, 'Beta', beta);

quat_est = zeros(sampleSize, 4);
euler_est = zeros(sampleSize,3);
for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));      % Update quaternion
    quat_est(t,:) = AHRS.Quaternion;
    euler_est(t,:) = quatern2euler(quat_est(t,:));
end

% Euler
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, euler_est(1:sampleSize,1), 'r');
plot(1:sampleSize, Euler(1:sampleSize,1), 'g');
title('Roll');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, euler_est(1:sampleSize,2), 'r');
plot(1:sampleSize, Euler(1:sampleSize,2), 'g');
title('Pitch');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, euler_est(1:sampleSize,3), 'r');
plot(1:sampleSize, Euler(1:sampleSize,3), 'g');
title('Yaw');
hold off;