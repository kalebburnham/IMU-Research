clc;
clear;
close all;
addpath('quaternion_library');

load('../data/logfile_web_straight.mat');
addpath('..');
addpath('../quaternion_library');


sampleSize = length(Acc);
period = 1/fs;
rollCorrection = 5*pi/180;
pitchCorrection = 15*pi/180;
yawCorrection = 0;

euler_est = zeros(sampleSize,3);
DCM_est = zeros(3,3,sampleSize);
quaternion = zeros(sampleSize, 4);

rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
InitialQuaternion = rotMat2quatern(rotm');

% Add error
noise = 1;
if noise
    load('../data/IMU_Xsens_noise.mat');
    Acc(:,1:3)=Acc(:,1:3)+Acc_noise(1:length(Acc),1:3);
    Gyr(:,1:3)=Gyr(:,1:3)+Gyr_noise(1:length(Gyr),1:3);
    Mag(:,1:3)=Mag(:,1:3)+Mag_noise(1:length(Mag),1:3);
end
for Kp = 0:0.1:1
    for Ki = 0:0.1:1
        
        AHRS = MahonyAHRS('SamplePeriod', period, 'Quaternion', InitialQuaternion,'Kp', Kp, 'Ki', Ki);
        
        for t = 1:sampleSize
            AHRS.Update(Gyr(t,:) , Acc(t,:), Mag(t,:));
            quaternion(t, :) = AHRS.Quaternion;
            DCM_est(:,:,t)=quatern2rotMat(quaternion(t,:))';  % always invert rotation matrix when using rotMat2quatern

            % Where do these come from? Different from quatern2euler
            euler_est(t,1)=(atan2(DCM_est(3,2,t),DCM_est(3,3,t)));
            euler_est(t,2)=(-asin(DCM_est(3,1,t)));
            euler_est(t,3)=(atan2(DCM_est(2,1,t),DCM_est(1,1,t)));
        end
        
        % Calculate Root Mean Square Error (Euler angles)
        RMSEMahonyEuler = sqrt(mean((Euler - euler_est).^2));
        disp(Kp);
        disp(Ki);
        disp(RMSEMahonyEuler);
        disp(norm(RMSEMahonyEuler));
        disp('-----------------');
    end
end