% Simulates an IMU located at [1 0 0] relative to the first IMU with no
% change in starting orientation.

%% Config
close all;
clear;
...clc;

load('../data/logfile_web_straight.mat');
addpath('../quaternion_library');
addpath('..');

period = 1/fs;
sampleSize = length(Acc);
start = 1;

%% Variable Declaration
DCM_est0 = zeros(3,3,sampleSize); DCM_est1 = zeros(3,3,sampleSize);
DCM_est2 = zeros(3,3,sampleSize); DCM_est3 = zeros(3,3,sampleSize);
quat_est0 = zeros(sampleSize,4);  quat_est1 = zeros(sampleSize,4);
quat_est2 = zeros(sampleSize,4);  quat_est3 = zeros(sampleSize,4);
euler_est = zeros(sampleSize,3); euler_est2 = zeros(sampleSize,3);
vel_est0 = zeros(sampleSize,3);   vel_est1 = zeros(sampleSize,3);
vel_est2 = zeros(sampleSize,3);   vel_est3 = zeros(sampleSize,3);
pos_est0 = zeros(sampleSize,3);   pos_est1 = zeros(sampleSize,3);
pos_est2 = zeros(sampleSize,3);   pos_est3 = zeros(sampleSize,3);
acc_est0 = zeros(sampleSize,3);   acc_est1 = zeros(sampleSize,3);
acc_est2 = zeros(sampleSize,3);   acc_est3 = zeros(sampleSize,3);
GyrAccel = zeros(sampleSize,3);

acc_est = zeros(n, sampleSize, 3);
vel_est = zeros(n, sampleSize, 3);
pos_est = zeros(n, sampleSize, 3);
euler_est = zeros(n, sampleSize, 3);
quat_est = zeros(n, sampleSize, 3);
DCM_est = zeros(3, 3, sampleSize, n);

%% Settings
rollCorrection = 5*pi/180;
pitchCorrection = 15*pi/180;
yawCorrection = 0;

%% Function

% Set offset parameters
% Position0 = [0 0 0];
% Position1 = [0 0 0];
% Position2 = [1 0 0];
% Position3 = [-1 0 0];

% Number of sensors
n = 4;
Position = [[0 0 0];
            [0 0 0];
            [1 0 0];
            [-1 0 0]];
groundTruthDCM = [[euler2rotMat(0, 0, 0)];
                  [euler2rotMat(pi, 0, pi/2)];
                  [euler2rotMat(0, 0, 0)];
                  [euler2rotMat(0, 0, 0)]];
              
% DCM0 = euler2rotMat(0, 0, 0);
% DCM1 = euler2rotMat(pi, 0, pi/2);
% DCM2 = euler2rotMat(0, 0, 0);
% DCM3 = euler2rotMat(0, 0, 0);

% Get d/dx of Gyr
Rot = zeros(size(Gyr));
for t = 2:length(Gyr)
    Rot(t,:) = (Gyr(t,:) - Gyr(t-1)) * period;
end

%% Rotate Data
RealAcc = zeros(n, length(Acc), 3);
RealGyr = zeros(n, length(Gyr), 3);
RealMag = zeros(n, length(Mag), 3);
for t = 1:n
    [RealAcc(n,:,:),RealGyr(n,:,:),RealMag(n,:,:)] = ApplyOffsets(Position(n,:), groundTruthDCM(n), Rot, Acc, Gyr, Mag);
%     [Acc1,Gyr1,Mag1] = ApplyOffsets(Position1, DCM1, Rot, Acc, Gyr, Mag);
%     [Acc2,Gyr2,Mag2] = ApplyOffsets(Position2, DCM2, Rot, Acc, Gyr, Mag);
%     [Acc3,Gyr3,Mag3] = ApplyOffsets(Position3, DCM3, Rot, Acc, Gyr, Mag);
end

%% Basic AHRS
rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
gamma = 1;

InitialQuaternion = zeros(n, 4);
AHRS = zeros(n, 1);

for t = 1:n
    InitialQuaternion(t,:) = rotMat2quatern((rotm * groundTruthDCM(n))');
%     InitialQuaternion0 = rotMat2quatern((rotm * DCM0)');
%     InitialQuaternion1 = rotMat2quatern((rotm * DCM1)');
%     InitialQuaternion2 = rotMat2quatern((rotm * DCM2)');
%     InitialQuaternion3 = rotMat2quatern((rotm * DCM3)');
    AHRS(t) = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion(t), 'Gamma', gamma);
%     AHRS0 = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion0, 'Gamma', gamma);
%     AHRS1 = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion1, 'Gamma', gamma);
%     AHRS2 = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion2, 'Gamma', gamma);
%     AHRS3 = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion3, 'Gamma', gamma);
end

DCM_est = zeros(3, 3, length(Acc), n); % Ground Truth DCM is (3,3,length(Acc));
quat_est = zeros(n, length(Acc), 4);
acc_est = zeros(n, length(Acc), 3);
vel_est = zeros(n, length(Acc), 3);
pos_est = zeros(n, length(Acc), 3);

% Function
for t = 1:sampleSize
    for num = 1:n
        AHRS(num).Update(RealGyr(num,t,:), RealAcc(num,t,3), RealMag(num,t,3));
        
%         AHRS0.Update(Gyr0(t,:), Acc0(t,:), Mag0(t,:));      % Update quaternion
%         AHRS1.Update(Gyr1(t,:), Acc1(t,:), Mag1(t,:));
%         AHRS0.Update(Gyr2(t,:), Acc2(t,:), Mag2(t,:));
%         AHRS3.Update(Gyr3(t,:), Acc3(t,:), Mag3(t,:));
        quat_est(n,t,:) = AHRS(num).Quaternion;
%         quat_est0(t,:) = AHRS0.Quaternion;
%         quat_est1(t,:) = AHRS1.Quaternion;
%         quat_est2(t,:) = AHRS2.Quaternion;
%         quat_est3(t,:) = AHRS3.Quaternion;


        DCM_est(:,:,t,num) = quatern2rotMat(quat_est(num,t,:));
%         DCM_est0(:,:,t) = quatern2rotMat(quat_est0(t,:))';  % always invert rotation matrix when using rotMat2quatern
%         DCM_est1(:,:,t) = quatern2rotMat(quat_est1(t,:))';
%         DCM_est2(:,:,t) = quatern2rotMat(quat_est2(t,:))';
%         DCM_est3(:,:,t) = quatern2rotMat(quat_est3(t,:))';

        % Rotate data to match global reference frame
        acc_est(num,t,:) = (DCM_est(:,:,t,num) * RealAcc(num,t,:)')';
%         acc_est0(t,:) = (DCM_est0(:,:,t)* Acc0(t,:)')';
%         acc_est1(t,:) = (DCM_est1(:,:,t)* Acc1(t,:)')';
%         acc_est2(t,:) = (DCM_est2(:,:,t)* Acc2(t,:)')';
%         acc_est3(t,:) = (DCM_est3(:,:,t)* Acc3(t,:)')';

        % Subtract Gravity
        acc_est(n,t,3) = acc_est(n,t,3) - gravity;
%         acc_est0(t,3) = acc_est0(t,3) - gravity;
%         acc_est1(t,3) = acc_est1(t,3) - gravity;
%         acc_est2(t,3) = acc_est2(t,3) - gravity;
%         acc_est3(t,3) = acc_est3(t,3) - gravity;
    end
end

%% Integrate
for t = 1000:sampleSize
    vel_est0(t,:) = vel_est0(t-1,:) + (acc_est0(t,:) * period); % Acc -> vel
    pos_est0(t,:) = pos_est0(t-1,:) + (vel_est0(t,:) * period); % vel -> pos
    
    vel_est1(t,:) = vel_est1(t-1,:) + (acc_est1(t,:) * period); % Acc -> vel
    pos_est1(t,:) = pos_est1(t-1,:) + (vel_est1(t,:) * period); % vel -> pos
    
    vel_est2(t,:) = vel_est2(t-1,:) + (acc_est2(t,:) * period); % Acc -> vel
    pos_est2(t,:) = pos_est2(t-1,:) + (vel_est2(t,:) * period); % vel -> pos
    
    vel_est3(t,:) = vel_est3(t-1,:) + (acc_est3(t,:) * period); % Acc -> vel
    pos_est3(t,:) = pos_est3(t-1,:) + (vel_est3(t,:) * period); % vel -> pos
end


%% Graphs
figure;
hold on;
plot(1:sampleSize, pos_est0(1:sampleSize,1), 'r');
plot(1:sampleSize, pos_est1(1:sampleSize,1), 'g');
plot(1:sampleSize, pos_est2(1:sampleSize,1), 'b');
plot(1:sampleSize, pos_est3(1:sampleSize,1), 'c');
plot(1:sampleSize, Pos(1:sampleSize,1), 'm');
title('Original Acceleration');
hold off;

figure;
hold on;
plot(1:sampleSize, pos_est0(1:sampleSize,2), 'r');
plot(1:sampleSize, pos_est1(1:sampleSize,2), 'g');
plot(1:sampleSize, pos_est2(1:sampleSize,2), 'b');
plot(1:sampleSize, pos_est3(1:sampleSize,2), 'c');
plot(1:sampleSize, Pos(1:sampleSize,2), 'm');
title('Original Acceleration');
hold off;

figure;
hold on;
plot(1:sampleSize, pos_est0(1:sampleSize,3), 'r');
plot(1:sampleSize, pos_est1(1:sampleSize,3), 'g');
plot(1:sampleSize, pos_est2(1:sampleSize,3), 'b');
plot(1:sampleSize, pos_est3(1:sampleSize,3), 'c');
plot(1:sampleSize, Pos(1:sampleSize,3), 'm');
title('Original Acceleration');
hold off;
