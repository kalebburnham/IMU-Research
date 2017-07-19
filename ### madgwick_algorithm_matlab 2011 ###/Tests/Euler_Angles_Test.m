close all;
clear;
clc;

addpath('../quaternion_library');
addpath('..');
load('../straight walk, 1000 steps.mat');

period = 1/fs;
beta = 0.08;
sampleSize = 2000;
pitchCorrection = 0.5790; % radians
rollCorrection = 0.0875;
yawCorrection = 0;

%% Initialization

% Find initial quaternion and create AHRS
rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', InitialQuaternion);

% Create variables
quaternion1 = zeros(sampleSize, 4);
quaternion2 = zeros(sampleSize, 4);
AbsAcc = zeros(sampleSize, 3); % Acceleration with respect to global axes - 'Absolute Acceleration'
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);

%% Function

% Calculate quaternions
for t = 1:sampleSize
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion1(t,:) = AHRS.Quaternion;
    
    % "use conjugate for sensor frame relative to Earth" - Madgwick
    quaternion1(t,:) = quaternConj(quaternion1(t,:));


    
    quaternion2(t,:) = dcm2quat(DCM(:,:,t));
end

figure;
hold on;
plot(1:sampleSize, quaternion1(1:sampleSize,1), 'r');
plot(1:sampleSize, quaternion2(1:sampleSize,1), 'g');
hold off;

figure;
hold on;
plot(1:sampleSize, quaternion1(1:sampleSize,2), 'r');
plot(1:sampleSize, quaternion2(1:sampleSize,2), 'g');
hold off;

figure;
hold on;
plot(1:sampleSize, quaternion1(1:sampleSize,3), 'r');
plot(1:sampleSize, quaternion2(1:sampleSize,3), 'g');
hold off;

figure;
hold on;
plot(1:sampleSize, quaternion1(1:sampleSize,4), 'r');
plot(1:sampleSize, quaternion2(1:sampleSize,4), 'g');
hold off;