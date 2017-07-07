close all;
clear;
clc;

addpath('../quaternion_library');
addpath('..');
load('../straight walk, 1000 steps.mat');

% This script tests if the rotation matrices are incorrect, using the
% ground truth DCMs to test for accuracy.

% This is done by testing that all samples. Since each matrix is an
% update on the previous, any drift should be blatant. Rotation matrices
% are converted to DCMs using Matlab's quat2dcm function. The calculated
% DCMs are then compared to the ground truth.

%% Settings
period = 1/100;
beta = .11;
sampleSize = 1500;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

%% Initialization

rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

quaternion = zeros(sampleSize, 4);
calculatedDCM = zeros(3, 3, sampleSize);

%% Function

for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    %quaternion(t,2) = -1 * quaternion(t,2);
    %quaternion(t,3) = -1 * quaternion(t,3);
    %quaternion(t,3) = -1 * quaternion(t,3);
    calculatedDCM(:,:,t) = quat2dcm(quaternion(t,:));
    
    % (Optional) Transpose calculatedDCM
    calculatedDCM(:,:,t) = permute(calculatedDCM(:,:,t), [2 1 3]);
end



%% Plot Results

A = squeeze(DCM(1,1,1:sampleSize)); % [1,1]
B = squeeze(DCM(1,2,1:sampleSize)); % [1,2]
C = squeeze(DCM(1,3,1:sampleSize)); % [1,3]
D = squeeze(DCM(2,1,1:sampleSize)); % [2,1]
E = squeeze(DCM(2,2,1:sampleSize)); % [2,2]
F = squeeze(DCM(2,3,1:sampleSize)); % [2,3]
G = squeeze(DCM(3,1,1:sampleSize)); % [3,1]
H = squeeze(DCM(3,2,1:sampleSize)); % [3,2]
I = squeeze(DCM(3,3,1:sampleSize)); % [3,3]

calculatedA = squeeze(calculatedDCM(1,1,1:sampleSize));
calculatedB = squeeze(calculatedDCM(1,2,1:sampleSize));
calculatedC = squeeze(calculatedDCM(1,3,1:sampleSize));
calculatedD = squeeze(calculatedDCM(2,1,1:sampleSize));
calculatedE = squeeze(calculatedDCM(2,2,1:sampleSize));
calculatedF = squeeze(calculatedDCM(2,3,1:sampleSize));
calculatedG = squeeze(calculatedDCM(3,1,1:sampleSize));
calculatedH = squeeze(calculatedDCM(3,2,1:sampleSize));
calculatedI = squeeze(calculatedDCM(3,3,1:sampleSize));

figure;
hold on;
title('[1,1]');
plot(1:sampleSize, A(:,1), 'r');
plot(1:sampleSize, calculatedA(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[1,2]');
plot(1:sampleSize, B(:,1), 'r');
plot(1:sampleSize, calculatedB(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[1,3]');
plot(1:sampleSize, C(:,1), 'r');
plot(1:sampleSize, calculatedC(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[2,1]');
plot(1:sampleSize, D(:,1), 'r');
plot(1:sampleSize, calculatedD(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[2,2]');
plot(1:sampleSize, E(:,1), 'r');
plot(1:sampleSize, calculatedE(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[2,3]');
plot(1:sampleSize, F(:,1), 'r');
plot(1:sampleSize, calculatedF(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[3,1]');
plot(1:sampleSize, G(:,1), 'r');
plot(1:sampleSize, calculatedG(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[3,2]');
plot(1:sampleSize, H(:,1), 'r');
plot(1:sampleSize, calculatedH(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[3,3]');
plot(1:sampleSize, I(:,1), 'r');
plot(1:sampleSize, calculatedI(:,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

