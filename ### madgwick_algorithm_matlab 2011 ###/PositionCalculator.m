clc;
clear;
close all;
addpath('quaternion_library');

...file = 'logfile_web_straight.mat';
file = 'Closed trajectory, 10 loops.mat';
%       straight walk, 1000 steps.mat - 122002 samples
%       logfile_web_straight.mat      - 22000 samples
%       Closed trajectory, 1 loop.mat - 11616 samples
%       Closed trajectory, 10 loops.mat     - 98160 samples

load(file);

%% Settings
period = 1/fs;
sampleSize = 22000; % Must be greater than 1000
first = 1000;

if strcmp(file,'straight walk, 1000 steps.mat') ...
        || strcmp(file, 'Closed trajectory, 1 loop.mat') ...
        || strcmp(file, 'Closed trajectory, 10 loops.mat')
    
    beta = 0.08;
    gravity = [0 0 -9.786];
    pitchCorrection = 0.5790; % radians
    rollCorrection = 0.0875;
    yawCorrection = 0;
    
elseif strcmp(file, 'logfile_web_straight.mat')
    beta = 0.075;
    gravity = [0 0 -9.8];
    pitchCorrection = 0.261799;
    rollCorrection = 0.0872665;
    yawCorrection = 0;
end
%% Initialization

% Find initial quaternion and create AHRS
rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', InitialQuaternion);

% Create variables
quaternion = zeros(sampleSize, 4);
AbsAcc = zeros(sampleSize, 3); % Acceleration with respect to global axes - 'Absolute Acceleration'
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);

%% Function

% Find acceleration in global frame
for t = 1:sampleSize
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    
    % "use conjugate for sensor frame relative to Earth" - Madgwick
    quaternion(t,:) = quaternConj(quaternion(t,:));
    
    % Rotate acceleration to Earth's frame and compensate for gravity
    AbsAcc(t,:) = quatrotate(quaternion(t,:), Acc(t,:)) + gravity;
    
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

% Integrate for velocity and position
for t = first+1:sampleSize
    vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
    pos(t,:) = pos(t-1,:) + (vel(t,:) * period);    % vel -> pos
end

%% Plot results

% Velocity
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,1), 'r');
title('Acceleration in Earths frame. X-Axis');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,2), 'r');
title('Acceleration in Earths frame. Y-Axis');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,3), 'r');
title('Acceleration in Earths frame. Z-Axis');
hold off;

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

figure;
hold on;
scatter(Mag(1:sampleSize,1), Mag(1:sampleSize,2));
scatter(Mag(1:sampleSize,1), Mag(1:sampleSize,3));
scatter(Mag(1:sampleSize,2), Mag(1:sampleSize,3));