%% Config
close all;
clear;
clc;

load('../data/logfile_web_straight.mat');
addpath('../quaternion_library');
addpath('..');

period = 1/fs;
sampleSize = 20000;
%sampleSize = length(Acc);
start = 1;

%% Variable Declaration
GyrAccel = zeros(sampleSize,3);

%% Settings
rollCorrection = 5*pi/180;
pitchCorrection = 15*pi/180;
yawCorrection = 0;

%% Function

% Set offset parameters
numOfSensors = 4;
Position = [[0 0 0];
            [0 0 0];
            [1 0 0];
            [-1 0 0]];
groundTruthDCM = zeros(3,3,numOfSensors);
groundTruthDCM(:,:,1) = euler2rotMat(0, 0, 0);    % Make these declarations cleaner
groundTruthDCM(:,:,2) = euler2rotMat(pi, 0, pi/2);
groundTruthDCM(:,:,3) = euler2rotMat(0, 0, 0);
groundTruthDCM(:,:,4) = euler2rotMat(0, 0, 0);

% Get d/dx of Gyr
Rot = zeros(size(Gyr));
for t = 2:length(Gyr)
    Rot(t,:) = (Gyr(t,:) - Gyr(t-1)) * period;
end

%% Variable declaration
DCM_est = zeros(3, 3, length(Acc), numOfSensors); % Ground Truth DCM is (3,3,length(Acc));
quat_est = zeros(numOfSensors, length(Acc), 4);
acc_est = zeros(numOfSensors, length(Acc), 3);
vel_est = zeros(numOfSensors, length(Acc), 3);
pos_est = zeros(numOfSensors, length(Acc), 3);

RealAcc = zeros(numOfSensors, length(Acc), 3); % Find a better name for these
RealGyr = zeros(numOfSensors, length(Gyr), 3);
RealMag = zeros(numOfSensors, length(Mag), 3);

%% Setup
rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
gamma = 1;

InitialQuaternion = zeros(numOfSensors, 4);
AHRS(1,numOfSensors) = BasicAHRS();

for n = 1:numOfSensors
    % Rotate Data
    [RealAcc(n,:,:),RealGyr(n,:,:),RealMag(n,:,:)] = ApplyOffsets(Position(n,:), groundTruthDCM(:,:,n), Rot, Acc, Gyr, Mag);
    
    InitialQuaternion(n,:) = rotMat2quatern(rotm * groundTruthDCM(:,:,n))';
    AHRS(n,:) = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion(n,:), 'Gamma', gamma);
end

%% Basic AHRS

% Function
for t = 1:sampleSize
    for n = 1:numOfSensors
        % Need 1x3 values instead of 1x1x3
        Gyroscope = squeeze(RealGyr(n,t,:))';
        Accelerometer = squeeze(RealAcc(n,t,:))';
        Magnetometer = squeeze(RealMag(n,t,:))';
        
        AHRS(n).Update(Gyroscope, Accelerometer, Magnetometer);
        quat_est(n,t,:) = AHRS(n).Quaternion;
        
        DCM_est(:,:,t,n) = quatern2rotMat(quat_est(n,t,:))'; % always invert rotation matrix when using rotMat2quatern

        % Rotate data to match global reference frame
        dcm_temp = DCM_est(:,:,t,n); % Need to reduce a dimension
        acc_temp = squeeze(RealAcc(n,t,:));
        acc_est(n,t,:) = (dcm_temp * acc_temp)';

        % Subtract Gravity
        acc_est(n,t,3) = acc_est(n,t,3) - gravity;
    end
end

%% Integrate
for t = 1000:sampleSize
    for n = 1:numOfSensors
        vel_est(n,t,:) = vel_est(n,t-1,:) + (acc_est(n,t,:) * period);
        pos_est(n,t,:) = pos_est(n,t-1,:) + (vel_est(n,t,:) * period);
    end
end


%% Graphs
color = ['g', 'b', 'c', 'y', 'm', 'w', 'k'];

% Position on global x-axis
figure;
hold on;
for n = 1:numOfSensors
    plot(1:sampleSize, pos_est(n,1:sampleSize,1), color(n));
end
plot(1:sampleSize, Pos(1:sampleSize,1), 'r');
title('Position X');
hold off;