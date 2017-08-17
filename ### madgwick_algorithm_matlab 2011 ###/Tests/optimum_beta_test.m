clc;
clear;
close all;
addpath('quaternion_library');

load('../straight walk, 1000 steps.mat');
addpath('..');
addpath('../quaternion_library');


sampleSize = 5000;
period = 1/fs;
gravity = [0 0 -9.786];
pitchCorrection = 0.5790; % radians
rollCorrection = 0.0875;
yawCorrection = 0;


smallestError = 100000000;
optimalBeta = 2;
for beta = 0:0.01:1

    % Find initial quaternion and create AHRS
    rotm = euler2rotMat(rollCorrection, pitchCorrection, yawCorrection);
    InitialQuaternion = rotMat2quatern(rotm);
    AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', InitialQuaternion);

    % Create variables
    quaternion = zeros(sampleSize, 4);
    GlobalAcc = zeros(sampleSize, 3); % Acceleration with respect to global axes
    vel_est = zeros(sampleSize, 3); % lower case = test results, not ground truth
    pos_est = zeros(sampleSize, 3);
    euler_est = zeros(size(Euler));

    [Acc,Gyr] = addError(Acc,Gyr);

    % Find acceleration in global frame
    for t = 1:sampleSize
        AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));      % Update quaternion
        quaternion(t,:) = AHRS.Quaternion;
        quaternion(t,:) = quaternConj(quaternion(t,:)); % "use conjugate for sensor frame relative to Earth" - Madgwick
        GlobalAcc(t,:) = quatrotate(quaternion(t,:), Acc(t,:)); % Rotate acceleration to Earth's frame
        GlobalAcc(t,:) = GlobalAcc(t,:) + gravity; % Compensate for gravity

        euler_est(t,:) = quatern2euler(quaternion(t,:));
    end

    % Integrate for velocity and position
    first = 1000;
    for t = first:sampleSize
        vel_est(t,:) = vel_est(t-1,:) + (GlobalAcc(t,:) * period); % Acc -> vel
        pos_est(t,:) = pos_est(t-1,:) + (vel_est(t,:) * period);    % vel -> pos
    end

    totalError = sqrt(pos_est(end,1)^2 + pos_est(end,2)^2 + pos_est(end,3)^2);
    
    if totalError < smallestError
        smallestError = totalError;
        optimalBeta = beta;
    end
    
end

disp("Optimum Beta Level: ");
disp(optimalBeta);
disp("With an error of ");
disp(smallestError);
disp("After");
disp(sampleSize);
disp("samples");