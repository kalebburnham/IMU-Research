close all;
clear;
clc;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Settings
period = 1/256;
sampleSize = 30000;
beta = 1;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
...Quaternion = [0.957466321359859 -0.041939572590074 -0.28520737125312 0.012492841766914];
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

%% Create Euler Angles and Acceleration
quaternion = zeros(sampleSize, 4);
euler = zeros(sampleSize, 3);
acc = zeros(sampleSize, 3);

for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    
    % To get the directions to match up on the plots, the new angles need
    % to be multiplied by -1. I don't know why this isn't done somewhere in
    % the filter.
    euler(t,:) = quatern2euler(quaternion(t,:));
    ...euler(t,:) = -1*quatern2euler(quaternion(t,:));

    % Acceleration on the global reference frame is calculated by
    % the accelerometer readings time the inverse of the rotation matrix.
    % Matlab says it's quicker to "divide" the matrix.
    acc(t,:) = Acc(t,:) * inv(quatern2rotMat(quaternion(t,:)));
    acc(t,:) = inv(quatern2rotMat(quaternion(t,:))) * Acc(t,:)';
    
end