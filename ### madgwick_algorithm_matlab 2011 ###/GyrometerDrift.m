close all;
clc;
clear;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Settings
period = 1/256; % per second
sampleSize = 122002;
pitchCorrection = 0.0875;
rollCorrection = -0.5790;
yawCorrection = 0;

%% Function

orientation = zeros(sampleSize, 3);
orientation(1, :) = [pitchCorrection(1,1), rollCorrection(1,1), yawCorrection(1,1)];

% Discrete Integration
for t = 2:sampleSize
    orientation(t,:) = orientation(t-1,:) + (Gyr(t,:) * period);
end

%% Plot results
t = [1:sampleSize]' * period;

figure('Name', 'Gyrometer Drift');
axis(1) = subplot(3,1,1);
hold on;
plot([0,t(end)], [pitchCorrection, pitchCorrection], 'r');
plot(t, orientation(:,1), 'g');
legend('Starting Orientation', 'Estimated Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('theta');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot([0,t(end)], [rollCorrection, rollCorrection], 'r');
plot(t, orientation(:,2), 'g');
legend('Starting Orientation', 'Estimated Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('phi');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot([0,t(end)], [yawCorrection, yawCorrection], 'r');
plot(t, orientation(:,3), 'g');
legend('Starting Orientation', 'Estimated Orientation');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('psi');
hold off;
linkaxes(axis, 'x');

%% NOTES

% Find drift between points by sum(Gyr(P1:P2), X);