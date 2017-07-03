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
period = 1/256;
beta = 5;
sampleSize = 122001;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

%% Initialization
first = length(DCM) - sampleSize;
last = length(DCM);

rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

quaternion = zeros(length(Acc), 4);
euler = zeros(length(Acc), 3);
rotm = zeros(3, 3, length(Acc));
calculatedDCM = zeros(3, 3, length(Acc));

%% Function

for t = 1:length(Acc)
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    calculatedDCM(:,:,t) = quat2dcm(quaternion(t,:));
end

%% (Optional) Transpose calculatedDCM
...calculatedDCM = permute(calculatedDCM, [2 1 3]);

%% Plot Results

A = squeeze(DCM(1,1,first:last)); % [1,1]
B = squeeze(DCM(1,2,first:last)); % [1,2]
C = squeeze(DCM(1,3,first:last)); % [1,3]
D = squeeze(DCM(2,1,first:last)); % [2,1]
E = squeeze(DCM(2,2,first:last)); % [2,2]
F = squeeze(DCM(2,3,first:last)); % [2,3]
G = squeeze(DCM(3,1,first:last)); % [3,1]
H = squeeze(DCM(3,2,first:last)); % [3,2]
I = squeeze(DCM(3,3,first:last)); % [3,3]

calculatedA = squeeze(calculatedDCM(1,1,:));
calculatedB = squeeze(calculatedDCM(1,2,:));
calculatedC = squeeze(calculatedDCM(1,3,:));
calculatedD = squeeze(calculatedDCM(2,1,:));
calculatedE = squeeze(calculatedDCM(2,2,:));
calculatedF = squeeze(calculatedDCM(2,3,:));
calculatedG = squeeze(calculatedDCM(3,1,:));
calculatedH = squeeze(calculatedDCM(3,2,:));
calculatedI = squeeze(calculatedDCM(3,3,:));

figure;
hold on;
title('[1,1]');
plot(first:last, A(:,1), 'r');
plot(first:last, calculatedA(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[1,2]');
plot(first:last, B(:,1), 'r');
plot(first:last, calculatedB(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[1,3]');
plot(first:last, C(:,1), 'r');
plot(first:last, calculatedC(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[2,1]');
plot(first:last, D(:,1), 'r');
plot(first:last, calculatedD(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[2,2]');
plot(first:last, E(:,1), 'r');
plot(first:last, calculatedE(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[2,3]');
plot(first:last, F(:,1), 'r');
plot(first:last, calculatedF(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[3,1]');
plot(first:last, G(:,1), 'r');
plot(first:last, calculatedG(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[3,2]');
plot(first:last, H(:,1), 'r');
plot(first:last, calculatedH(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

figure;
hold on;
title('[3,3]');
plot(first:last, I(:,1), 'r');
plot(first:last, calculatedI(first:last,1), 'g');
legend('Ground Truth', 'Calculated');
hold off;

