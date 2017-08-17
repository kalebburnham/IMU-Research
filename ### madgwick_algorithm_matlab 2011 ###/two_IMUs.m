addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

load('data/logfile_web_straight.mat');

period = 1/fs;
sampleSize = length(Acc);
start = 1000;
%% Plots
plot_1 = 'yes'; % Position Esimate
plot_2 = 'yes';


%%
AccReversed = Acc;% .* -1;
GyrReversed = Gyr;% .* -1;
MagReversed = Mag;% .* -1;

vel_est1 = zeros(sampleSize, 3); 
pos_est1 = zeros(sampleSize, 3);
euler_est1 = zeros(sampleSize, 3);
AbsAcc1 = zeros(sampleSize, 3); % Absolute Acceleration - acceleration with respect to global axes
DCM_est1 = zeros(3,3,sampleSize); % Direction Cosine Matrix
vel_est2 = zeros(sampleSize, 3); 
pos_est2 = zeros(sampleSize, 3);
euler_est2 = zeros(sampleSize, 3);
AbsAcc2 = zeros(sampleSize, 3); % Absolute Acceleration - acceleration with respect to global axes
DCM_est2 = zeros(3,3,sampleSize); % Direction Cosine Matrix

rollCorrection1 = 5*pi/180;      % in radians
pitchCorrection1 = 15*pi/180;    % in radians
yawCorrection1 = 0;              % in radians
rollCorrection2 = 5*pi/180;
pitchCorrection2 = 15*pi/180;
yawCorrection2 = 180*pi/180;

rotm1 = euler2rotMat(rollCorrection1, pitchCorrection1, yawCorrection1);
rotm2 = euler2rotMat(rollCorrection2, pitchCorrection2, yawCorrection2);
InitialQuaternion1 = rotMat2quatern(rotm1');
InitialQuaternion2 = rotMat2quatern(rotm2');
AHRS1 = MadgwickAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion1, 'Beta', 0.3); % 1/256, 'Beta', 0.1);
AHRS2 = MadgwickAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion2, 'Beta', 0.3);
quaternion1 = zeros(sampleSize, 4);
quaternion2 = zeros(sampleSize, 4);
for t = 1:sampleSize
    AHRS1.Update(Gyr(t,:), Acc(t,:), Mag(t,:));	% gyroscope units must be radians
    AHRS2.Update(GyrReversed(t,:), AccReversed(t,:), MagReversed(t,:));
    quaternion1(t,:) = AHRS1.Quaternion;
    quaternion2(t,:) = AHRS2.Quaternion;

    DCM_est1(:,:,t)=quatern2rotMat(quaternion1(t,:))';  % always invert rotation matrix when using rotMat2quatern
    DCM_est2(:,:,t)=quatern2rotMat(quaternion2(t,:))';

    % Register Euler estimations
    euler_est1(t,1)=(atan2(DCM_est1(3,2,t),DCM_est1(3,3,t)));  % Roll
    euler_est1(t,2)=(-asin(DCM_est1(3,1,t)));  % Pitch
    euler_est1(t,3)=(atan2(DCM_est1(2,1,t),DCM_est1(1,1,t)));  %Yaw
    euler_est2(t,1)=(atan2(DCM_est2(3,2,t),DCM_est2(3,3,t)));  % Roll
    euler_est2(t,2)=(-asin(DCM_est2(3,1,t)));  % Pitch
    euler_est2(t,3)=(atan2(DCM_est2(2,1,t),DCM_est2(1,1,t)));  %Yaw

    % Rotate data to match global reference frame
    AbsAcc1(t,:) = (DCM_est1(:,:,t) * Acc(t,:)')';
    AbsAcc2(t,:) = (DCM_est2(:,:,t) * AccReversed(t,:)')';

    % Subtract Gravity
    AbsAcc1(t,3) = AbsAcc1(t,3) - gravity;
    AbsAcc2(t,3) = AbsAcc2(t,3) + gravity;
end

for t = 2:sampleSize
    vel_est1(t,:) = vel_est1(t-1,:) + (AbsAcc1(t,:) * period); % Acc -> vel
    pos_est1(t,:) = pos_est1(t-1,:) + (vel_est1(t,:) * period); % vel -> pos
    vel_est2(t,:) = vel_est2(t-1,:) + (AbsAcc2(t,:) * period);
    pos_est2(t,:) = pos_est2(t-1,:) + (vel_est2(t,:) * period);
end

if strcmp(plot_1, 'yes')
    figure;
    axis(1) = subplot(3,1,1);
    hold on;
    plot(start:sampleSize, pos_est1(start:sampleSize,1), 'r');
    plot(start:sampleSize, pos_est2(start:sampleSize,1), 'g');
    plot(start:sampleSize, Pos(start:sampleSize,1), 'b');
    title('Position Estimate on X-axis');
    legend('Estimate', 'Estimate2', 'Ground Truth');
    hold off;
    axis(2) = subplot(3,1,2);
    hold on;
    plot(start:sampleSize, pos_est1(start:sampleSize,2), 'r');
    plot(start:sampleSize, pos_est2(start:sampleSize,2), 'g');
    plot(start:sampleSize, Pos(start:sampleSize,2), 'b');
    title('Position Estimate on Y-axis');
    legend('Estimate', 'Estimate2', 'Ground Truth');
    hold off;
    axis(3) = subplot(3,1,3);
    hold on;
    plot(start:sampleSize, pos_est1(start:sampleSize,3), 'r');
    plot(start:sampleSize, pos_est2(start:sampleSize,3), 'g');
    plot(start:sampleSize, Pos(start:sampleSize,3), 'b');
    title('Position Estimate on Z-axis');
    legend('Estimate', 'Estimate3', 'Ground Truth');
    hold off;
    set(gcf,'Color',[1 1 1])
end

% if strcmp(plot_2, 'yes')
%     figure; hold off; 
%     % Fit Euler to +/- 180 degrees
%     Euler(:,1)=atan2(sin(Euler(:,1)),cos(Euler(:,1)));
%     Euler(:,2)=atan2(sin(Euler(:,2)),cos(Euler(:,2)));
%     Euler(:,3)=atan2(sin(Euler(:,3)),cos(Euler(:,3)));
%     subplot(3,1,1);
%     ax=1; plot(start:sampleSize,euler_est2(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
%     title(strcat('Euler estimate correctness - ', {' '}, INS_Method)); ylabel('Roll (degrees)');
%     subplot(3,1,2);
%     ax=2; plot(start:sampleSize,euler_est2(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
%     ylabel('Pitch (degrees)');
%     subplot(3,1,3);
%     ax=3; plot(start:sampleSize,euler_est2(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
%     ylabel('Yaw (degrees)');
%     xlabel('samples');
%     legend({'Estimate','Real'});  
%     set(gcf,'Color',[1 1 1])
% end