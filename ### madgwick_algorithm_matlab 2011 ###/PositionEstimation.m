% PositionEstimation.m

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Plots
plot_1 = 'no'; % Sensor data

%% Import and plot sensor data
load('straight walk, 1000 steps.mat');
%load('logfile_web_straight.mat');
%load('logfile_web_3loops.mat');

rowsDeleteEnd=0; %16000; %16000; %20000; %1000;
Acc=Acc(1:end-rowsDeleteEnd,:);
Mag=Mag(1:end-rowsDeleteEnd,:);
Gyr=Gyr(1:end-rowsDeleteEnd,:);
sampleSize = 20000;
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
pitchCorrection = 30*pi/180;    % in radians
yawCorrection = 0;              % in radians
gravity = 9.786;
INS_Method = 'Madgwick'; % Choose one: 'Madgwick', 'Mahony', 'Basic'

% Plot sensor data
if strcmp(plot_1, 'yes')
    figure('Name', 'Sensor Data');
    axis(1) = subplot(3,1,1);
    hold on;
    plot(1:sampleSize, Gyr(1:sampleSize,1), 'r');
    plot(1:sampleSize, Gyr(1:sampleSize,2), 'g');
    plot(1:sampleSize, Gyr(1:sampleSize,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    hold off;
    axis(2) = subplot(3,1,2);
    hold on;
    plot(1:sampleSize, Acc(1:sampleSize,1), 'r');
    plot(1:sampleSize, Acc(1:sampleSize,2), 'g');
    plot(1:sampleSize, Acc(1:sampleSize,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;
    axis(3) = subplot(3,1,3);
    hold on;
    plot(1:sampleSize, Mag(1:sampleSize,1), 'r');
    plot(1:sampleSize, Mag(1:sampleSize,2), 'g');
    plot(1:sampleSize, Mag(1:sampleSize,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    linkaxes(axis, 'x');
end

% Add Noise
[Acc, Gyr] = addError(Acc, Gyr);

%% Process sensor data via Madgwick AHRS
if strcmp(INS_Method, 'Madgwick')
    AHRS = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', .08);
    quaternion1 = zeros(sampleSize, 4);
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));            % Update quaternion
        quaternion1(t,:) = AHRS.Quaternion;
        quaternion1(t,:) = quaternConj(quaternion1(t,:));     % "use conjugate for sensor frame relative to Earth" - Madgwick
        AbsAcc(t,:) = quatrotate(quaternion1(t,:), Acc(t,:)); % Rotate data to match global reference frame
        AbsAcc(t,3) = AbsAcc(t,3) - gravity;                  % Subtract Gravity
        
        % Register Euler angles
        euler_est(t,:) = quatern2euler(quaternion1(t,:));
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
        euler_est(t,:) = rotMat2euler(DCM_est(:,:,t));

        % Rotate data to match global reference frame
        AbsAcc(t,:) = Acc(t,:) * DCM_est(:,:,t);
        %AbsAcc(t,:) = (DCM_est(:,:,t)* Acc(t,:)')';

        % Subtract Gravity
        AbsAcc(t,3) = AbsAcc(t,3) - gravity;
    end
end

start=1000; %1000;
% Integrate for velocity and position
for t = start+1:sampleSize
    vel_est(t,:) = vel_est(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
    pos_est(t,:) = pos_est(t-1,:) + (vel_est(t,:) * period); % vel -> pos
end



%% Plot results

% 2D position
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, pos_est(1:sampleSize,1), 'r');
plot(1:sampleSize, Pos(1:sampleSize,1), 'b');
title('Position Estimate on X-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, pos_est(1:sampleSize,2), 'r');
plot(1:sampleSize, Pos(1:sampleSize,2), 'b');
title('Position Estimate on Y-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, pos_est(1:sampleSize,3), 'r');
plot(1:sampleSize, Pos(1:sampleSize,3), 'b');
title('Position Estimate on Z-axis');
legend('Estimate', 'Ground Truth');
hold off;
set(gcf,'Color',[1 1 1])

% 3D position
figure;
axis(1) = subplot(2,1,1);
title('Position Estimate');
plot3(pos_est(1:sampleSize,1), pos_est(1:sampleSize,2), pos_est(1:sampleSize,3), 'r');
axis(2) = subplot(2,1,2);
plot3(Pos(1:sampleSize,1), Pos(1:sampleSize,2), Pos(1:sampleSize,3), 'b');
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
ax=1; plot(1:sampleSize,euler_est(1:sampleSize,ax)*180/pi,'r-'); hold on; plot(1:sampleSize,Euler(1:sampleSize,ax)*180/pi,'b-');
title('Euler estimate correctness'); ylabel('Roll (degrees)');
subplot(3,1,2);
ax=2; plot(1:sampleSize,euler_est(1:sampleSize,ax)*180/pi,'r-'); hold on; plot(1:sampleSize,Euler(1:sampleSize,ax)*180/pi,'b-');
ylabel('Pitch (degrees)');
subplot(3,1,3);
ax=3; plot(1:sampleSize,euler_est(1:sampleSize,ax)*180/pi,'r-'); hold on; plot(1:sampleSize,Euler(1:sampleSize,ax)*180/pi,'b-');
ylabel('Yaw (degrees)');
xlabel('samples');
legend({'Estimate','Real'});  
set(gcf,'Color',[1 1 1])

%% End of script