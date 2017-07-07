clc;
clear;
close all;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Settings
period = 1/100;
sampleSize = 10000;
beta = 0.08;
gravity = 9.786; % m/s/s
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

%% Initialization

% Find initial quaternion and create AHRS
rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

%% Function
quaternion = zeros(sampleSize, 4);
AbsAcc = zeros(sampleSize, 3);
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth

for t = 1:sampleSize
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    
    % Perform weird adjustment
    quaternion(t,2:4) = -1 * quaternion(t,2:4);
    
    % Rotate data to match global reference frame
    AbsAcc(t,:) = Acc(t,:) * inv(quatern2rotMat(quaternion(t,:)));
        
    % Subtract Gravity
    AbsAcc(t,3) = AbsAcc(t,3) - gravity;
end

% Integrate acceleration for velocity
for t = 1000:sampleSize
    vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period);
end

%% Plot Results

figure;
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,1), 'r');
title('Absolute Acceleration on X-axis');
legend('X');
hold off;

figure;
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,2), 'r');
title('Absolute Acceleration on Y-axis');
legend('Y');
hold off;

figure;
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,3), 'r');
title('Absolute Acceleration on Z-axis');
legend('Z');
hold off;

figure;
hold on;
plot(1:sampleSize, vel(1:sampleSize,1), 'r');
plot(1:sampleSize, Vel(1:sampleSize,1), 'g');
title('Velocity (X)');
legend('Estimated', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, vel(1:sampleSize,2), 'r');
plot(1:sampleSize, Vel(1:sampleSize,2), 'g');
title('Velocity (Y)');
legend('Estimated', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, vel(1:sampleSize,3), 'r');
plot(1:sampleSize, Vel(1:sampleSize,3), 'g');
title('Velocity (Z)');
legend('Estimated', 'Ground Truth');
hold off;