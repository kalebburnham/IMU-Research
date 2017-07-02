close all;
clear;
clc;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Settings
period = 1/256; % per second
sampleSize = 10000;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

%% Function

euler = zeros(sampleSize, 3);
euler(1, :) = [pitchCorrection(1,1), rollCorrection(1,1), yawCorrection(1,1)];

% Discrete Integration
for t = 2:sampleSize
    euler(t,:) = euler(t-1,:) + (Gyr(t,:) * period);
end

%% Plot results
t = [1:sampleSize]' * period;

figure('Name', 'Gyrometer Drift');
axis(1) = subplot(3,1,1);
hold on;
plot([0,t(end)], [pitchCorrection, pitchCorrection], 'r');
plot(t, euler(:,1), 'g');
legend('Starting Orientation', 'Estimated Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('theta');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot([0,t(end)], [rollCorrection, rollCorrection], 'r');
plot(t, euler(:,2), 'g');
legend('Starting Orientation', 'Estimated Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('phi');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot([0,t(end)], [yawCorrection, yawCorrection], 'r');
plot(t, euler(:,3), 'g');
legend('Starting Orientation', 'Estimated Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('psi');
hold off;
linkaxes(axis, 'x');