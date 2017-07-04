close all;
clear;
clc;

addpath('../quaternion_library');
addpath('..');
load('../straight walk, 1000 steps.mat');

% This tests if the calculated quaternions and the quaternions from the 
% ground truth DCMs are different.

%% Settings
period = 1/256;
beta = 1;
sampleSize = 2000;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

%% Initialization

rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

quaternion = zeros(length(Acc), 4);
groundTruthQuat = zeros(length(Acc), 4);

%% Function

for t = 1:length(Acc)
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    groundTruthQuat(t,:) = dcm2quat(DCM(:,:,t));
end

%% Plot Quaternions

figure;
hold on;
plot(1:sampleSize, quaternion(1:sampleSize,1), 'r');
plot(1:sampleSize, groundTruthQuat(1:sampleSize,1), 'g');
title('W');
legend('Estimated', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, quaternion(1:sampleSize,2), 'r');
plot(1:sampleSize, groundTruthQuat(1:sampleSize,2), 'g');
title('X');
legend('Estimated', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, quaternion(1:sampleSize,3), 'r');
plot(1:sampleSize, groundTruthQuat(1:sampleSize,3), 'g');
title('Y');
legend('Estimated', 'Ground Truth');
hold off;

figure;
hold on;
plot(1:sampleSize, quaternion(1:sampleSize,4), 'r');
plot(1:sampleSize, groundTruthQuat(1:sampleSize,4), 'g');
title('Z');
legend('Estimated', 'Ground Truth');
hold off;