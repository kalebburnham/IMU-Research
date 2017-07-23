clc;
clear;
close all;
addpath('quaternion_library');

...file = 'logfile_web_straight.mat';
file = 'straight walk, 1000 steps.mat';
%       straight walk, 1000 steps.mat - 122002 samples
%       logfile_web_straight.mat      - 22000 samples
%       Closed trajectory, 1 loop.mat - 11616 samples
%       Closed trajectory, 10 loops.mat     - 98160 samples

load(file);

%% Settings
period = 1/fs;
sampleSize = 20000; % Must be greater than 1000
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
    beta = 0.08;
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
GlobalAcc = zeros(sampleSize, 3); % Acceleration with respect to global axes
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);
euler_est = zeros(size(Euler));

%% Function

%[Acc,Gyr] = addError(Acc,Gyr);

% Find acceleration in global frame
for t = 1:sampleSize
    
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    
    % "use conjugate for sensor frame relative to Earth" - Madgwick
    quaternion(t,:) = quaternConj(quaternion(t,:));
     
    % Rotate acceleration to Earth's frame and compensate for gravity
    GlobalAcc(t,:) = quatrotate(quaternion(t,:), Acc(t,:)) + gravity;
    
    euler_est(t,:) = quatern2euler(quaternion(t,:));
    
%     cap = 0.2;
%     if abs(GlobalAcc(t,1)) < cap
%         GlobalAcc(t,1) = 0;
%     end
%     
%     if abs(GlobalAcc(t,2)) < cap
%         GlobalAcc(t,2) = 0;
%     end
%     
%     if abs(GlobalAcc(t,3)) < cap
%         GlobalAcc(t,3) = 0;
%     end
        
end

% Integrate for velocity and position
for t = first+1:sampleSize
%     if norm(AbsAcc(t,:)) == 0
%         vel(t,:) = [0 0 0];
%     else
%         vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
%     end
    
    vel(t,:) = vel(t-1,:) + (GlobalAcc(t-1,:) * period); % Acc -> vel
    pos(t,:) = pos(t-1,:) + (vel(t-1,:) * period);    % vel -> pos
end

GroundTruthAcceleration = zeros(sampleSize,3);
for t = first:sampleSize-1
    GroundTruthAcceleration(t,:) = (Vel(t+1,:) - Vel(t,:)) * 1/period;
end

figure;
hold on;
plot(1024:2000, GlobalAcc(1024:2000,3), 'r');
plot(1024:2000, GroundTruthAcceleration(1024:2000,3), 'g');
hold off;


%% Plot results
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

% Velocity
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, GlobalAcc(1:sampleSize,1), 'r');
title('Acceleration in Earths frame. X-Axis');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, GlobalAcc(1:sampleSize,2), 'r');
title('Acceleration in Earths frame. Y-Axis');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, GlobalAcc(1:sampleSize,3), 'r');
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

%3D Position
figure;
hold on;
axis(1) = subplot(2,1,1);
plot3(pos(1:sampleSize,1), pos(1:sampleSize,2), pos(1:sampleSize,3), 'r');
title('Position Estimate');
axis(2) = subplot(2,1,2);
plot3(Pos(1:sampleSize,1), Pos(1:sampleSize,2), Pos(1:sampleSize,3), 'r');
title('Position Ground Truth');
hold off;

figure;
scatter(1:1001, GlobalAcc(1000:2000,3))

