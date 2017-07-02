close all;
clc;
clear;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Settings
period = 1/256;
sampleSize = 10000;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

% To find the initial quaternion, convert the Euler angles 
% [0.0875, 0.5790 0] to a rotation matrix and the rotation matrix to a
% quaternion using the library functions.
Quaternion = [0.957466321359859 -0.041939572590074 -0.28520737125312 0.012492841766914];
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', 1, 'Quaternion', Quaternion);

%% Function
quaternion = zeros(sampleSize, 4);
euler = zeros(sampleSize, 3);
for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    
    % To get the directions to match up on the plots, the new angles need
    % to be multiplied by -1. I don't know why this isn't done somewhere in
    % the filter.
    euler(t,:) = quatern2euler(quaternion(t,:));
    ...euler(t,:) = -1*quatern2euler(quaternion(t,:));
end

%% Plot results
t = [1:sampleSize]' * period;

figure('Name', 'Euler Drift');
axis(1) = subplot(3,1,1);
hold on;
plot(t, euler(:,1), 'b');
plot(t, Euler(1:sampleSize,1), 'g');
plot([0,t(end)], [pitchCorrection, pitchCorrection], 'r');
legend('Estimated Orientation', 'Ground Truth Euler', 'Starting Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Pitch');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(t, euler(:,2), 'b');
plot(t, Euler(1:sampleSize,2), 'g');
plot([0,t(end)], [rollCorrection, rollCorrection], 'r');
legend('Estimated Orientation', 'Ground Truth Euler', 'Starting Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Roll');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(t, euler(:,3), 'b');
plot(t, Euler(1:sampleSize,3), 'g');
plot([0,t(end)], [yawCorrection, yawCorrection], 'r');
legend('Estimated Orientation', 'Ground Truth Euler', 'Starting Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Yaw');
hold off;
linkaxes(axis, 'x');

