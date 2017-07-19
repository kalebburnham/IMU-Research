clc;
clear;
close all;
addpath('quaternion_library');

file = 'Closed trajectory, 1 loop.mat';
%       straight walk, 1000 steps.mat - 122002 samples
%       logfile_web_straight.mat      - 22000 samples
%       Closed trajectory, 1 loop.mat - 11616 samples
%       Closed trajectory, 10.mat     - 98160 samples

load(file);

%% Settings
period = 1/fs;
sampleSize = 11616; % Must be greater than start
start = 1; % AHRS provides unreliable values at the beginning

if strcmp(file,'straight walk, 1000 steps.mat') ...
        || strcmp(file, 'Closed trajectory, 1 loop.mat') ...
        || strcmp(file, 'Closed trajectory, 10 loops.mat')
    gravity = [0 0 -9.786];
elseif strcmp(file, 'logfile_web_straight.mat')
    gravity = [0 0 -9.8];
end

%% Initialization

% Create variables
AbsAcc = zeros(sampleSize, 3); % Ground truth acceleration in earth's frame w/out gravity
vel = zeros(sampleSize, 3); % lower case = test results
pos = zeros(sampleSize, 3);

Quaternion = dcm2quat(DCM(:,:,:)); % Calculate ground truth quaternions

for t = 1:sampleSize
    % Rotate to Earth's frame and remove the gravity vector.
    AbsAcc(t,:) = quatrotate(Quaternion(t,:), Acc(t,:)) + gravity;
    
    % Eliminate trivial values. This is done to eliminate noise when the
    % object is at rest and Matlab precision errors that cause a tiny drift
    % on the y axis on the order of 10^-16. It makes the graph look bad
    % since the axes scale.
    cap = 0.1;
    if abs(AbsAcc(t,1)) < cap
        AbsAcc(t,1) = 0;
    end
    
    if abs(AbsAcc(t,2)) < cap
        AbsAcc(t,2) = 0;
    end
    
    if abs(AbsAcc(t,3)) < cap
        AbsAcc(t,3) = 0;
    end
end

% Integration
for t = start+1:sampleSize
    vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
    pos(t,:) = pos(t-1,:) + (vel(t,:) * period);    % vel -> pos
end

%% Graphs

% 2D results
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, pos(1:sampleSize,1), 'r');
plot(1:sampleSize, Pos(1:sampleSize,1), 'g');
title('Position Estimate on X-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, pos(1:sampleSize,2), 'r');
plot(1:sampleSize, Pos(1:sampleSize,2), 'g');
title('Position Estimate on Y-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, pos(1:sampleSize,3), 'r');
plot(1:sampleSize, Pos(1:sampleSize,3), 'g');
title('Position Estimate on Z-axis');
legend('Estimate', 'Ground Truth');
hold off;

% 3D Position
figure;
axis(1) = subplot(2,1,1);
plot3(pos(1:sampleSize,1), pos(1:sampleSize,2), pos(1:sampleSize,3), 'r');
title('Position Estimate');
axis(2) = subplot(2,1,2);
plot3(Pos(1:sampleSize,1), Pos(1:sampleSize,2), Pos(1:sampleSize,3), 'r');
title('Position Ground Truth');
