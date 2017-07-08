clc;
clear;
close all;
addpath('quaternion_library');
...load('straight walk, 1000 steps.mat'); % 122002 samples
...load('Closed trajectory, 1 loop.mat'); % 11616 samples
load('Closed trajectory, 10 loops.mat'); % 98160 samples

%% Settings
period = 1/100;
sampleSize = 98160;
beta = .08;
gravity = 9.786; % m/s/s
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

%% Initialization

% Find initial quaternion and create AHRS
rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

% Create variables
quaternion = zeros(sampleSize, 4);
AbsAcc = zeros(sampleSize, 3);
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);

%% Function

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

% Integrate velocity for position
for t = 1000:sampleSize
    pos(t,:) = pos(t-1,:) + (vel(t,:) * period);
end

%% Plot results
figure;
hold on;
plot(1:sampleSize, pos(1:sampleSize,1), 'r');
plot(1:sampleSize, Pos(1:sampleSize,1), 'g');
title('Position Estimate on X-axis');
legend('Estimate', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, pos(1:sampleSize,2), 'r');
plot(1:sampleSize, Pos(1:sampleSize,2), 'g');
title('Position Estimate on Y-axis');
legend('Estimate', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, pos(1:sampleSize,3), 'r');
plot(1:sampleSize, Pos(1:sampleSize,3), 'g');
title('Position Estimate on Z-axis');
legend('Estimate', 'Ground Truth');
hold off;

figure;
hold on;
scatter3(pos(1:sampleSize,1), pos(1:sampleSize,2), pos(1:sampleSize,3));
hold off;