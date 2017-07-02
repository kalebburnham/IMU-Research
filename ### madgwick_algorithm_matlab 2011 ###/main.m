addpath('quaternion_library');
clear;
clc;
close all;

load('straight walk, 1000 steps.mat');

period = 1/fs;
sampleSize = 1105;
AHRS = MadgwickAHRS('SamplePeriod', 1/100, 'Beta', 0.1, 'Quaternion', [0.957466321359859 -0.041939572590074 -0.28520737125312 0.012492841766914]);
% AHRS = MahonyAHRS('SamplePeriod', 1/100, 'Kp', 0.5);

%% Deal with gravity
 % Rotates the Accelerometer data so it reads [0 0 9.7860] at rest.
 % Performs two 2-dimensional rotations. I don't know how 
 % to convert it to a single 3-dimensional rotation.
 
AccRotated = zeros(sampleSize, 3);

% There are unique values for each row.

% Should test later on to see if the drift is significant.
theta = .58078;     % Pitch correction. Gives 0 for x value
phi = -0.07325;   % Roll correction. Gives 0 for y value.
...theta = 0.5790;  % Pitch correction value on spec sheet
...phi = -0.0875; % Roll correction value on spec sheet

% 2D Rotation matrices from Wikipedia
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
R2 = [cos(phi) -sin(phi); sin(phi) cos(phi)];
    
% Rotate all the values in Acc and put them in AccRotated
% Can probably just change Acc eventually

% AccRotated = Acc*rotm^-1

for t = 1:length(sampleSize)
    % Rotate to make x = 0;
    v = [Acc(t,1) Acc(t,3)];
    vR = v*R;
    AccRotated(t,1) = vR(1);
    AccRotated(t,3) = vR(2);
    
    % Rotate to make y = 0;
    v = [Acc(t,2) AccRotated(t,3)];
    vR2 = v*R2;
    AccRotated(t,2) = vR2(1);
    AccRotated(t,3) = vR2(2);
end

% Subtract Gravity
AccRotated(:,3) = AccRotated(:,3) - 9.7860;


% Questions:
% 1) Does gravity need to be removed?
% 2) Should the AccRotated vector at rest read [0 0 -9.8760]?
% 3) How accurate do theta and theta2 need to be?

%% Create Quaternions and Rotation Matrices
quaternion = zeros(length(sampleSize), 4);
rotMat = zeros(length(sampleSize), 3, 3);
for t = 1:length(Acc)
    ...AHRS.Update(Gyr(t,:), AccRotated(t,:), Mag(t,:));
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));	% gyroscope units must be radians/sec
    quaternion(t, :) = AHRS.Quaternion;
    rotMat = quatern2rotMat(quaternion(t,:));
    
    AccRotated(t,:) = Acc(t,:) / rotMat(:,:);
end




%% Integrate acceleration to find position
vel = zeros(size(AccRotated));
for t = 2:length(vel)
   vel(t,:) = vel(t-1,:) + Acc(t,:) * period;
end
