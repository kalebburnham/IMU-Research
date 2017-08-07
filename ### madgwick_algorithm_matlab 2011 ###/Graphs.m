close all;
clc;
clear;

load('logfile_web_straight.mat');
addpath('quaternion_library');

period = 1/fs;
rollCorrection = 5*pi/180;      % in radians
pitchCorrection = 15*pi/180;    % in radians
yawCorrection = 0;              % in radians
sampleSize = length(Acc);
gravity = 9.8;

start = 1000;
finish = 3000;

%% Calculate Madgwick Euler Estimation
MadgwickOrientation = zeros(size(Gyr));
madgwick_DCM_est = zeros(3,3,sampleSize);
madgwick_euler_est = zeros(sampleSize, 3);
madgwick_global_acc = zeros(sampleSize, 3);
rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm');
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', 0.1, 'Quaternion', InitialQuaternion); % 1/256, 'Beta', 0.1);
quaternion1 = zeros(sampleSize, 4);

for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:)); % Gyr units are already in rad/s
    quaternion1(t,:) = AHRS.Quaternion;
    
    madgwick_DCM_est(:,:,t)=quatern2rotMat(quaternion1(t,:))';  % always invert rotation matrix when using rotMat2quatern

    % Register Euler estimations
    madgwick_euler_est(t,1)=(atan2(madgwick_DCM_est(3,2,t),madgwick_DCM_est(3,3,t)));  % Roll
    madgwick_euler_est(t,2)=(-asin(madgwick_DCM_est(3,1,t)));  % Pitch
    madgwick_euler_est(t,3)=(atan2(madgwick_DCM_est(2,1,t),madgwick_DCM_est(1,1,t)));  %Yaw
    
    % Rotate data to match global reference frame
    madgwick_global_acc(t,:) = (madgwick_DCM_est(:,:,t) * Acc(t,:)')';

    % Subtract Gravity
    madgwick_global_acc(t,3) = madgwick_global_acc(t,3) - gravity;
end

madgwick_vel_est = zeros(sampleSize, 3); 
madgwick_pos_est = zeros(sampleSize, 3);
for t = start+1:sampleSize
    madgwick_vel_est(t,:) = madgwick_vel_est(t-1,:) + (madgwick_global_acc(t,:) * period); % Acc -> vel
    madgwick_pos_est(t,:) = madgwick_pos_est(t-1,:) + (madgwick_vel_est(t,:) * period); % vel -> pos
end

% Calculate Root Mean Square Error
RMSE_Madgwick_Euler = sqrt(mean((Euler(start:finish,:) - madgwick_euler_est(start:finish,:)).^2))
RMSE_Madgwick_Pos = sqrt(mean((Pos(start:finish,:) - madgwick_pos_est(start:finish,:)).^2))
        
%% Calculate Mahony Euler Estimation
MahonyOrientation = zeros(size(Gyr));
mahony_DCM_est = zeros(3,3,sampleSize);
mahony_euler_est = zeros(sampleSize,3);
mahony_global_acc = zeros(sampleSize, 3);
AHRS = MahonyAHRS('SamplePeriod', period, 'Kp', 0.3, 'Quaternion', InitialQuaternion);
quaternion2 = zeros(sampleSize, 4);
for t = 1:sampleSize
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion2(t,:) = AHRS.Quaternion;
    
    mahony_DCM_est(:,:,t) = quatern2rotMat(quaternion2(t,:))';
    
    mahony_euler_est(t,1)=(atan2(mahony_DCM_est(3,2,t),mahony_DCM_est(3,3,t)));  % Roll
    mahony_euler_est(t,2)=(-asin(mahony_DCM_est(3,1,t)));  % Pitch
    mahony_euler_est(t,3)=(atan2(mahony_DCM_est(2,1,t),mahony_DCM_est(1,1,t)));  %Yaw
    
     % Rotate data to match global reference frame
    mahony_global_acc(t,:) = (mahony_DCM_est(:,:,t) * Acc(t,:)')';

    % Subtract Gravity
    mahony_global_acc(t,3) = mahony_global_acc(t,3) - gravity;
end

mahony_vel_est = zeros(sampleSize, 3); 
mahony_pos_est = zeros(sampleSize, 3);
for t = start+1:sampleSize
    mahony_vel_est(t,:) = mahony_vel_est(t-1,:) + (mahony_global_acc(t,:) * period); % Acc -> vel
    mahony_pos_est(t,:) = mahony_pos_est(t-1,:) + (mahony_vel_est(t,:) * period); % vel -> pos
end

RMSE_Mahony_Euler = sqrt(mean((Euler(start:finish,:) - mahony_euler_est(start:finish,:)).^2))
RMSE_Mahony_Pos = sqrt(mean((Pos(start:finish,:) - mahony_pos_est(start:finish,:)).^2))


figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(start:finish, madgwick_euler_est(start:finish,1)*180/pi, 'r');
plot(start:finish, mahony_euler_est(start:finish,1)*180/pi, 'g');
plot(start:finish, Euler(start:finish,1)*180/pi, 'b');
title('Orientation Estimation');
ylabel('Roll (degrees)');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(start:finish, madgwick_euler_est(start:finish,2)*180/pi, 'r');
plot(start:finish, mahony_euler_est(start:finish,2)*180/pi, 'g');
plot(start:finish, Euler(start:finish,2)*180/pi, 'b');
ylabel('Pitch (degrees)');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(start:finish, madgwick_euler_est(start:finish,3)*180/pi, 'r');
plot(start:finish, mahony_euler_est(start:finish,3)*180/pi, 'g');
plot(start:finish, Euler(start:finish,3)*180/pi, 'b');
ylabel('Yaw (degrees)');
xlabel('Sample Number');
legend('Madgwick', 'Mahony', 'Ground Truth');
hold off;

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(start:finish, madgwick_pos_est(start:finish,1), 'r');
plot(start:finish, mahony_pos_est(start:finish,1), 'g');
plot(start:finish, Pos(start:finish,1), 'b');
title('Position Estimation');
ylabel('X (m)');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(start:finish, madgwick_pos_est(start:finish,2), 'r');
plot(start:finish, mahony_pos_est(start:finish,2), 'g');
plot(start:finish, Pos(start:finish,2), 'b');
ylabel('Y (m)');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(start:finish, madgwick_pos_est(start:finish,3), 'r');
plot(start:finish, mahony_pos_est(start:finish,3), 'g');
plot(start:finish, Pos(start:finish,3), 'b');
xlabel('Sample Number');
ylabel('Z (m)');
legend('Madgwick', 'Mahony', 'Ground Truth');
hold off;
