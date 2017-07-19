close all;
clear;
clc;

addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

gravity = [0 0 -9.786];
period = 1/fs;
sampleSize = 1500;
sigma = 0.012;

n = 3;
H = [1 0 0;
     0 1 0;
     0 0 1];

% Differentiate for Velocity and Acceleration
velocity = zeros(sampleSize,3);
worldAcceleration = zeros(sampleSize,3);
for t = 2:sampleSize
    velocity(t,:) = (Pos(t,:) - Pos(t-1,:)) * fs;
    worldAcceleration(t,:) = (velocity(t,:) - velocity(t-1,:)) * fs;
end


%% Convert DCM to rotm and multiply by GroundTruthAcceleration. Add gravity

% Find Ground Truth Quaternions
Quat = zeros(sampleSize, 4);
Rotm = zeros(3,3,sampleSize);
localAccelerationWithGrav = zeros(size(worldAcceleration));
for t = 1:sampleSize
    Quat(t,:) = dcm2quat(DCM(:,:,t));
    localAccelerationWithGrav(t,:) = worldAcceleration(t,:) - gravity;
    localAccelerationWithGrav(t,:) = localAccelerationWithGrav(t,:) * quatern2rotMat(Quat(t,:));
end

vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);

for t = 2:sampleSize
    vel(t,:) = vel(t-1,:) + (worldAcceleration(t,:) * period); % Acc -> vel
    pos(t,:) = pos(t-1,:) + (vel(t,:) * period);    % vel -> pos
end

%% Generate noise
AccNoise = normrnd(0,sigma,sampleSize,3);




