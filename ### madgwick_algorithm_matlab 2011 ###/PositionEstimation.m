% PositionEstimation.m

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Plots
plot_1 = 'yes'; % Sensor data

%% Other settings
removeTrivialValues = false;

%% Import and plot sensor data
load('logfile_web_straight.mat');
%load('logfile_web_3loops.mat');

rowsDeleteEnd=0; %16000; %16000; %20000; %1000;
Acc=Acc(1:end-rowsDeleteEnd,:);
Mag=Mag(1:end-rowsDeleteEnd,:);
Gyr=Gyr(1:end-rowsDeleteEnd,:);
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
INS_Method = 'Madgwick'; % Choose one: 'Madgwick', 'Mahony', 'Basic'

% Plot sensor data
if strcmp(plot_1, 'yes')
    figure('Name', 'Sensor Data');
    axis(1) = subplot(3,1,1);
    hold on;
    plot(1:sampleSize, Gyr(:,1), 'r');
    plot(1:sampleSize, Gyr(:,2), 'g');
    plot(1:sampleSize, Gyr(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    hold off;
    axis(2) = subplot(3,1,2);
    hold on;
    plot(1:sampleSize, Acc(:,1), 'r');
    plot(1:sampleSize, Acc(:,2), 'g');
    plot(1:sampleSize, Acc(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;
    axis(3) = subplot(3,1,3);
    hold on;
    plot(1:sampleSize, Mag(:,1), 'r');
    plot(1:sampleSize, Mag(:,2), 'g');
    plot(1:sampleSize, Mag(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    linkaxes(axis, 'x');
end


%% Process sensor data via Madgwick AHRS
if strcmp(INS_Method, 'Madgwick')
    AHRS = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', 0.08); % 1/256, 'Beta', 0.1);
    quaternion1 = zeros(sampleSize, 4);
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:)); % It's already in radians??
        %AHRS.Update(Gyr(t,:) * (pi/180), Acc(t,:), Mag(t,:));	% gyroscope units must be radians
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
end


%% Process sensor data via Mahony AHRS
if strcmp(INS_Method, 'Mahony')
    AHRS = MahonyAHRS('SamplePeriod', 1/fs, 'Kp', 0.5); %'Kp', 0.5);
    quaternion2 = zeros(sampleSize, 4);
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
        %AHRS.Update(Gyr(t,:) * (pi/180), Acc(t,:), Mag(t,:));	% gyroscope units must be radians
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

end

%% Process sensor data through basic method using only Gyro data
if strcmp(INS_Method, 'Basic')
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
end

% Remove trivial values
if removeTrivialValues
    for t = 1:sampleSize
        if AbsAcc(t,1) < 0.1
            AbsAcc(t,1) = 0;
        end
        if AbsAcc(t,2) < 0.1
            AbsAcc(t,2) = 0;
        end
        if AbsAcc(t,3) < 0.1
            AbsAcc(t,3) = 0;
        end
    end
end


start=1000; %1000;
% Integrate for velocity and position
if strcmp(integrationMethod, 'Rectangular')
    for t = start+1:sampleSize
        vel_est(t,:) = vel_est(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
        pos_est(t,:) = pos_est(t-1,:) + (vel_est(t,:) * period); % vel -> pos
    end
elseif strcmp(integrationMethod, 'Trapezoidal')
    for t = start+1:sampleSize
        vel_est(t,:) = vel_est(t-1,:) + (AbsAcc(t-1,:) + AbsAcc(t,:)) * 0.5 * period; % Acc -> vel
        pos_est(t,:) = pos_est(t-1,:) + (vel_est(t-1,:) + vel_est(t,:)) * 0.5 * period; % vel -> pos
    end
end


%% Plot results

% 2D position
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

% 3D position
figure;
axis(1) = subplot(2,1,1);
plot3(pos_est(start:sampleSize,1), pos_est(start:sampleSize,2), pos_est(start:sampleSize,3), 'r');
title('Position Estimate'); 
axis(2) = subplot(2,1,2);
plot3(Pos(start:sampleSize,1), Pos(start:sampleSize,2), Pos(start:sampleSize,3), 'b');
title('Position Ground Truth');
set(gcf,'Color',[1 1 1])
if strcmp(INS_Method, 'Basic')
    axH = findall(gcf,'type','axes');
    set(axH,'ylim',[-0.05 0.05], 'xlim',[1 150], 'zlim', [0 0.1])
end

% Check Euler estimate correctness
figure; hold off; 
% Fit Euler to +/- 180 degrees
Euler(:,1)=atan2(sin(Euler(:,1)),cos(Euler(:,1)));
Euler(:,2)=atan2(sin(Euler(:,2)),cos(Euler(:,2)));
Euler(:,3)=atan2(sin(Euler(:,3)),cos(Euler(:,3)));
subplot(3,1,1);
ax=1; plot(start:sampleSize,euler_est(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
title('Euler estimate correctness'); ylabel('Roll (degrees)');
subplot(3,1,2);
ax=2; plot(start:sampleSize,euler_est(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
ylabel('Pitch (degrees)');
subplot(3,1,3);
ax=3; plot(start:sampleSize,euler_est(start:sampleSize,ax)*180/pi,'r-'); hold on; plot(start:sampleSize,Euler(start:sampleSize,ax)*180/pi,'b-');
ylabel('Yaw (degrees)');
xlabel('samples');
legend({'Estimate','Real'});  
set(gcf,'Color',[1 1 1])

%% End of script