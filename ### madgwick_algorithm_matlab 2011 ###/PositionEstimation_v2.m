% PositionEstimation.m

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
...clc;                                % clear the command terminal

% Choose options:
INS_Method = 'BasicAHRS'; % Choose one: 'Madgwick', 'Mahony', 'Gyro', 'BasicAHRS'
noise=0;    % 1: noise  0: noiseless

%% Plots
plot_1 = 'no'; % Sensor data
plot_2 = 'yes'; % 2D Position
plot_3 = 'no'; % 3D Position
plot_4 = 'yes'; % Euler estimation correctness

tic
%% Import and plot sensor data
load('data/logfile_web_straight.mat');  % Units are in meters, seconds and radians. Sampling frequency: 100 Hz
%load('logfile_web_3loops.mat');

rowsDeleteEnd=0; 16000; %16000; %20000; %1000;
Acc=Acc(1:end-rowsDeleteEnd,:);   % in meter/second^2
Mag=Mag(1:end-rowsDeleteEnd,:);
Gyr=Gyr(1:end-rowsDeleteEnd,:);  % in radiants/second
sampleSize = length(Acc);
Pos=Pos(1:end-rowsDeleteEnd,:);
Vel=Vel(1:end-rowsDeleteEnd,:);
DCM=DCM(1:end-rowsDeleteEnd,:);
Euler=Euler(1:end-rowsDeleteEnd,:);
dcm=DCM(1:end-rowsDeleteEnd,:);
period = 1/fs;

% Create variables for estimations
vel_est = zeros(sampleSize, 3); 
pos_est = zeros(sampleSize, 3);
euler_est = zeros(sampleSize, 3);
AbsAcc = zeros(sampleSize, 3); % Absolute Acceleration - acceleration with respect to global axes
DCM_est = zeros(3,3,sampleSize); % Direction Cosine Matrix

rollCorrection = 5*pi/180;      % in radians
pitchCorrection = 15*pi/180;    % in radians
yawCorrection = 0;              % in radians
integrationMethod = 'Rectangular'; % Rectangular or Trapezoidal

if noise
    load('data/IMU_Xsens_noise.mat');
    Acc(:,1:3)=Acc(:,1:3)+Acc_noise(1:length(Acc),1:3);
    Gyr(:,1:3)=Gyr(:,1:3)+Gyr_noise(1:length(Gyr),1:3);
    Mag(:,1:3)=Mag(:,1:3)+Mag_noise(1:length(Mag),1:3);
end

%% Process sensor data via Basic AHRS
if strcmp(INS_Method, 'BasicAHRS')
    
    rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
    InitialQuaternion = rotMat2quatern(rotm');
    gamma=1; %[0-1]  0: only Acc-Mag       1: only Gyro  0.5: in between
    AHRS = BasicAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion, 'Gamma', gamma);
    quat_est = zeros(sampleSize, 4);
    euler_est = zeros(sampleSize,3);
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));      % Update quaternion
        quat_est(t,:) = AHRS.Quaternion;

        DCM_est(:,:,t) = quatern2rotMat(quat_est(t,:))';  % always invert rotation matrix when using rotMat2quatern

        % Register Euler estimations
        euler_est(t,1)=(atan2(DCM_est(3,2,t),DCM_est(3,3,t)));  % Roll
        euler_est(t,2)=(-asin(DCM_est(3,1,t)));  % Pitch
        euler_est(t,3)=(atan2(DCM_est(2,1,t),DCM_est(1,1,t)));  %Yaw

        % Rotate data to match global reference frame
        AbsAcc(t,:) = (DCM_est(:,:,t)* Acc(t,:)')';

        % Subtract Gravity
        AbsAcc(t,3) = AbsAcc(t,3) - gravity;
    end
    % Calculate Root Mean Square Error (Euler angles)
    RMSEBasicAHRSEuler = sqrt(mean((Euler - euler_est).^2))
        
end


%% Process sensor data via Madgwick AHRS
if strcmp(INS_Method, 'Madgwick')
    
    rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
    InitialQuaternion = rotMat2quatern(rotm');
    AHRS = MadgwickAHRS('SamplePeriod', period,'Quaternion', InitialQuaternion, 'Beta', 0.3); % 1/256, 'Beta', 0.1);
    quaternion1 = zeros(sampleSize, 4);
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));	% gyroscope units must be radians
        quaternion1(t,:) = AHRS.Quaternion;

        DCM_est(:,:,t)=quatern2rotMat(quaternion1(t,:))';  % always invert rotation matrix when using rotMat2quatern

        % Register Euler estimations
        euler_est(t,1)=(atan2(DCM_est(3,2,t),DCM_est(3,3,t)));  % Roll
        euler_est(t,2)=(-asin(DCM_est(3,1,t)));  % Pitch
        euler_est(t,3)=(atan2(DCM_est(2,1,t),DCM_est(1,1,t)));  %Yaw

        % Rotate data to match global reference frame
        AbsAcc(t,:) = (DCM_est(:,:,t)* Acc(t,:)')';

        % Subtract Gravity
        AbsAcc(t,3) = AbsAcc(t,3) - gravity;
    end
    % Calculate Root Mean Square Error (Euler angles)
    RMSEMadgwickEuler = sqrt(mean((Euler - euler_est).^2))
        
end


%% Process sensor data via Mahony AHRS
if strcmp(INS_Method, 'Mahony')
    Kp = 0;
    rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
    InitialQuaternion = rotMat2quatern(rotm');
    AHRS = MahonyAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion,'Kp', Kp); %'Kp', 0.5);
    quaternion2 = zeros(sampleSize, 4);
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:) , Acc(t,:), Mag(t,:));	% gyroscope units must be radians
        quaternion2(t, :) = AHRS.Quaternion;
        DCM_est(:,:,t)=quatern2rotMat(quaternion2(t,:))';  % always invert rotation matrix when using rotMat2quatern

        % Register Euler estimations
        euler_est(t,1)=(atan2(DCM_est(3,2,t),DCM_est(3,3,t)));  % Roll
        euler_est(t,2)=(-asin(DCM_est(3,1,t)));  % Pitch
        euler_est(t,3)=(atan2(DCM_est(2,1,t),DCM_est(1,1,t)));  %Yaw

        % Rotate data to match global reference frame
        AbsAcc(t,:) = (DCM_est(:,:,t)* Acc(t,:)')';

        % Subtract Gravity
        AbsAcc(t,3) = AbsAcc(t,3) - gravity;
    end
    % Calculate Root Mean Square Error (Euler angles)
    RMSEMahonyEuler = sqrt(mean((Euler - euler_est).^2))
end


%% Process sensor data through basic method using only Gyro data
if strcmp(INS_Method, 'Gyro')
    % Find initial quaternion and create AHRS
    rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
    quaternion3 = zeros(sampleSize, 4);
    for t = 1:sampleSize
        % Integrate to calculate quaternion using just gyroscope data
        if t>1
            w_norm=norm(Gyr(t,:));
            if w_norm>0
               q_w=[cos(w_norm/2*period),  sin(w_norm/2*period)*Gyr(t,1)/w_norm,  sin(w_norm/2*period)*Gyr(t,2)/w_norm, sin(w_norm/2*period)*Gyr(t,3)/w_norm ];
               quaternion3(t,:)=quaternProd(quaternion3(t-1,:),q_w);
            else
               quaternion3(t,:)=quaternion3(t-1,:);
            end
        else
            rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
            quaternion3(t,:)=rotMat2quatern(rotm'); % always invert rotation matrix when using rotMat2quatern
        end

        quaternion3(t,:) = quaternion3(t,:) / norm(quaternion3(t,:)); % normalize quaternion
        DCM_est(:,:,t)=quatern2rotMat(quaternion3(t,:))';  % always invert rotation matrix when using rotMat2quatern

        % Register Euler estimations
        euler_est(t,1)=(atan2(DCM_est(3,2,t),DCM_est(3,3,t)));  % Roll
        euler_est(t,2)=(-asin(DCM_est(3,1,t)));  % Pitch
        euler_est(t,3)=(atan2(DCM_est(2,1,t),DCM_est(1,1,t)));  %Yaw

        % Rotate data to match global reference frame
        AbsAcc(t,:) = (DCM_est(:,:,t)* Acc(t,:)')';

        % Subtract Gravity
        AbsAcc(t,3) = AbsAcc(t,3) - gravity;
    end
    % Calculate Root Mean Square Error (Euler angles)
    RMSEGyroEuler = sqrt(mean((Euler - euler_est).^2)) % *180/pi
end


% Integrate for velocity and position
if strcmp(integrationMethod, 'Rectangular')
    for t = 2:sampleSize
        vel_est(t,:) = vel_est(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
        pos_est(t,:) = pos_est(t-1,:) + (vel_est(t,:) * period); % vel -> pos
    end
elseif strcmp(integrationMethod, 'Trapezoidal')
    for t = 2:sampleSize
        vel_est(t,:) = vel_est(t-1,:) + (AbsAcc(t-1,:) + AbsAcc(t,:)) * 0.5 * period; % Acc -> vel
        pos_est(t,:) = pos_est(t-1,:) + (vel_est(t-1,:) + vel_est(t,:)) * 0.5 * period; % vel -> pos
    end
end


%% Plot results
start=1000; %1000;
finish=20000;  %3000;
sampleSize = sampleSize - finish;

% Plot sensor data
if strcmp(plot_1, 'yes')
    figure('Name', 'Sensor Data');
    axis(1) = subplot(3,1,1);
    hold on;
    plot(start:sampleSize, Gyr(start:sampleSize,1), 'r');
    plot(start:sampleSize, Gyr(start:sampleSize,2), 'g');
    plot(start:sampleSize, Gyr(start:sampleSize,3), 'b');
    for ii=start:sampleSize, norm_Gyr(ii-start+1)=norm(Gyr(ii,1:3)); end
    plot(start:sampleSize, norm_Gyr, 'k:');
    legend('X', 'Y', 'Z','norm');
    xlabel('Time (s)');
    ylabel('Angular rate (rad/s)');
    title('Gyroscope');
    hold off;
    axis(2) = subplot(3,1,2);
    hold on;
    plot(start:sampleSize, Acc(start:sampleSize,1), 'r');
    plot(start:sampleSize, Acc(start:sampleSize,2), 'g');
    plot(start:sampleSize, Acc(start:sampleSize,3), 'b');
    for ii=start:sampleSize, norm_Acc(ii-start+1)=norm(Acc(ii,1:3)); end
    plot(start:sampleSize, norm_Acc, 'k:');
    legend('X', 'Y', 'Z','norm');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s2)');
    title('Accelerometer');
    hold off;
    axis(3) = subplot(3,1,3);
    hold on;
    plot(start:sampleSize, Mag(start:sampleSize,1), 'r');
    plot(start:sampleSize, Mag(start:sampleSize,2), 'g');
    plot(start:sampleSize, Mag(start:sampleSize,3), 'b');
    for ii=start:sampleSize, norm_Mag(ii-start+1)=norm(Mag(ii,1:3)); end
    plot(start:sampleSize, norm_Mag, 'k:');
    legend('X', 'Y', 'Z','norm');
    xlabel('Time (s)');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    linkaxes(axis, 'x');
end

% Plot 2D position
if strcmp(plot_2, 'yes')
    figure;
    axis(1) = subplot(3,1,1);
    hold on;
    plot(start:sampleSize, pos_est(start:sampleSize,1), 'r');
    plot(start:sampleSize, Pos(start:sampleSize,1), 'b');
    title('Position Estimate on X-axis');
    legend('Estimate', 'Ground Truth');
    hold off;
    axis(2) = subplot(3,1,2);
    hold on;
    plot(start:sampleSize, pos_est(start:sampleSize,2), 'r');
    plot(start:sampleSize, Pos(start:sampleSize,2), 'b');
    title('Position Estimate on Y-axis');
    legend('Estimate', 'Ground Truth');
    hold off;
    axis(3) = subplot(3,1,3);
    hold on;
    plot(start:sampleSize, pos_est(start:sampleSize,3), 'r');
    plot(start:sampleSize, Pos(start:sampleSize,3), 'b');
    title('Position Estimate on Z-axis');
    legend('Estimate', 'Ground Truth');
    hold off;
    set(gcf,'Color',[1 1 1])
end

% Plot 3D position
if strcmp(plot_3, 'yes')
    figure;
    axis(1) = subplot(2,1,1);
    plot3(pos_est(start:sampleSize,1), pos_est(start:sampleSize,2), pos_est(start:sampleSize,3), 'r');
    title('Position Estimate'); 
    axis(2) = subplot(2,1,2);
    plot3(Pos(start:sampleSize,1), Pos(start:sampleSize,2), Pos(start:sampleSize,3), 'b');
    title('Position Ground Truth');
    set(gcf,'Color',[1 1 1])
    % if strcmp(INS_Method, 'Basic')
    %     axH = findall(gcf,'type','axes');
    %     set(axH,'ylim',[-0.5 0.5], 'xlim',[1 150], 'zlim', [0 0.1])
    % end
    % if strcmp(INS_Method, 'BasicAHRS')
    %     axH = findall(gcf,'type','axes');
    %     set(axH,'ylim',[-0.05 0.05], 'xlim',[185 200], 'zlim', [0 0.15])
    % end
end

% Check Euler estimate correctness
if strcmp(plot_4, 'yes')
    figure; hold off; 
    % Fit Euler to +/- 180 degrees
    Euler(:,1)=atan2(sin(Euler(:,1)),cos(Euler(:,1)));
    Euler(:,2)=atan2(sin(Euler(:,2)),cos(Euler(:,2)));
    Euler(:,3)=atan2(sin(Euler(:,3)),cos(Euler(:,3)));
    subplot(3,1,1);
    ax=1; plot(start:sampleSize,euler_est(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
    title(strcat('Euler estimate correctness - ', {' '}, INS_Method)); ylabel('Roll (degrees)');
    subplot(3,1,2);
    ax=2; plot(start:sampleSize,euler_est(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
    ylabel('Pitch (degrees)');
    subplot(3,1,3);
    ax=3; plot(start:sampleSize,euler_est(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
    ylabel('Yaw (degrees)');
    xlabel('samples');
    legend({'Estimate','Real'});  
    set(gcf,'Color',[1 1 1])
end

execTime = toc

%% End of script