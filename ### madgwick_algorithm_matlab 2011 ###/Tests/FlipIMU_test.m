close all;
clear;
clc;

load('../data/logfile_web_straight.mat');
addpath('../quaternion_library');
addpath('..');

period = 1/fs;
sampleSize = 1500;
start = 1;

% Create variables for estimations
DCM_est1 = zeros(3,3,sampleSize); % Direction Cosine Matrix
DCM_est2 = zeros(3,3,sampleSize);

rollCorrection = 5*pi/180;
pitchCorrection = 15*pi/180;
yawCorrection = 0;

rollCorrection2 = 5*pi/180;
pitchCorrection2 = 15*pi/180;
yawCorrection2 = 0;

Acc2(:,1) = Acc(:,1);
Acc2(:,2) = Acc(:,2);
Acc2(:,3) = -Acc(:,3);
Gyr2(:,1) = Gyr(:,1);
Gyr2(:,2) = Gyr(:,2);
Gyr2(:,3) = -Gyr(:,3);
Mag2(:,1) = Mag(:,1);
Mag2(:,2) = Mag(:,2);
Mag2(:,3) = -Mag(:,3);

rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm');
rotm2 = euler2rotMat(rollCorrection2, pitchCorrection2, yawCorrection2);
InitialQuaternion2 = rotMat2quatern(rotm2');

gamma=1;   
AHRS1 = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion, 'Gamma', gamma);
AHRS2 = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion2, 'Gamma', gamma);
quat_est1 = zeros(sampleSize, 4);
quat_est2 = zeros(sampleSize, 4);
AbsAcc1 = zeros(sampleSize, 3);
AbsAcc2 = zeros(sampleSize, 3);

for t = 1:sampleSize
    AHRS1.Update(Gyr(t,:), Acc(t,:), Mag(t,:));      % Update quaternion
    AHRS2.Update(Gyr2(t,:), Acc2(t,:), Mag2(t,:));
    quat_est1(t,:) = AHRS1.Quaternion;
    quat_est2(t,:) = AHRS2.Quaternion;

    DCM_est1(:,:,t) = quatern2rotMat(quat_est1(t,:))';  % always invert rotation matrix when using rotMat2quatern
    DCM_est2(:,:,t) = quatern2rotMat(quat_est2(t,:))';
    
    AbsAcc1(t,:) = (DCM_est1(:,:,t) * Acc(t,:)')';
    AbsAcc2(t,:) = (DCM_est2(:,:,t) * Acc2(t,:)')';
    
    AbsAcc1(t,3) = AbsAcc1(t,3) - gravity;
    AbsAcc2(t,3) = AbsAcc2(t,3) - gravity;
end

euler_est1 = quatern2euler(quat_est1);
euler_est2 = quatern2euler(quat_est2);

vel_est1 = zeros(sampleSize, 3); 
pos_est1 = zeros(sampleSize, 3);
vel_est2 = zeros(sampleSize, 3);
pos_est2 = zeros(sampleSize, 3);

for t = 1000:sampleSize
    vel_est1(t,:) = vel_est1(t-1,:) + (AbsAcc1(t,:) * period); % Acc -> vel
    pos_est1(t,:) = pos_est1(t-1,:) + (vel_est1(t,:) * period); % vel -> pos
    vel_est2(t,:) = vel_est2(t-1,:) + (AbsAcc2(t,:) * period); % Acc -> vel
    pos_est2(t,:) = pos_est2(t-1,:) + (vel_est2(t,:) * period); % vel -> pos
end

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(start:sampleSize, Acc(start:sampleSize,1), 'r');
plot(start:sampleSize, Acc2(start:sampleSize,1), 'b');
legend('Original', 'New');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(start:sampleSize, Acc(start:sampleSize,2), 'r');
plot(start:sampleSize, Acc2(start:sampleSize,2), 'b');
legend('Original', 'New');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(start:sampleSize, Acc(start:sampleSize,3), 'r');
plot(start:sampleSize, Acc2(start:sampleSize,3), 'b');
legend('Original', 'New');
hold off;
set(gcf,'Color',[1 1 1])

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(start:sampleSize, euler_est1(start:sampleSize,1), 'r');
plot(start:sampleSize, euler_est2(start:sampleSize,1), 'b');
legend('Original', 'New');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(start:sampleSize, euler_est1(start:sampleSize,2), 'r');
plot(start:sampleSize, euler_est2(start:sampleSize,2), 'b');
legend('Original', 'New');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(start:sampleSize, euler_est1(start:sampleSize,3), 'r');
plot(start:sampleSize, euler_est2(start:sampleSize,3), 'b');
legend('Original', 'New');
hold off;
set(gcf,'Color',[1 1 1])

% figure;
% axis(1) = subplot(3,1,1);
% hold on;
% plot(start:sampleSize, pos_est1(start:sampleSize,1), 'r');
% plot(start:sampleSize, pos_est2(start:sampleSize,1), 'b');
% plot(start:sampleSize, Pos(start:sampleSize,1), 'g');
% hold off;
% axis(2) = subplot(3,1,2);
% hold on;
% plot(start:sampleSize, pos_est1(start:sampleSize,2), 'r');
% plot(start:sampleSize, pos_est2(start:sampleSize,2), 'b');
% plot(start:sampleSize, Pos(start:sampleSize,2), 'g');
% hold off;
% axis(3) = subplot(3,1,3);
% hold on;
% plot(start:sampleSize, pos_est1(start:sampleSize,3), 'r');
% plot(start:sampleSize, pos_est2(start:sampleSize,3), 'b');
% plot(start:sampleSize, Pos(start:sampleSize,3), 'g');
% legend('Original', 'New', 'Ground Truth');
% hold off;
% set(gcf,'Color',[1 1 1])

