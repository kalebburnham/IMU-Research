close all;
clc;
clear;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Settings
period = 1/256;
sampleSize = 1500;
beta = 0;
gravity = 9.786; % m/s/s
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
...Quaternion = [0.957466321359859 -0.041939572590074 -0.28520737125312 0.012492841766914];
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);


%% Function
quaternion = zeros(sampleSize, 4);
AbsAcc = zeros(sampleSize, 3);
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    ...AbsAcc(t,:) = inv(quatern2rotMat(quaternion(t,:))) * Acc(t,:)';
    AbsAcc(t,:) = Acc(t,:) * inv(quatern2rotMat(quaternion(t,:)));
        
    % Subtract Gravity
    AbsAcc(t,3) = AbsAcc(t,3) - gravity;
end

% Integrate acceleration for velocity
for t = 2:sampleSize
    vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period);
end




% Plot Example Results

figure;
hold on;
plot(1:sampleSize, AbsAcc(1:sampleSize,1), 'r');
plot(1:sampleSize, AbsAcc(1:sampleSize,2), 'g');
plot(1:sampleSize, AbsAcc(1:sampleSize,3), 'b');
title('True Accelerations');
legend('X', 'Y', 'Z');
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