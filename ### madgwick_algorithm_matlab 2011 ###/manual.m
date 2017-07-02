close all;
clc;
clear;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

period = 1/256;
sampleSize = 1050;

%% Deal with gravity
 % Rotates the Accelerometer data so it reads [0 0 9.7860] at rest.
 % Performs two 2-dimensional rotations. I don't know how 
 % to convert it to a single 3-dimensional rotation.
 
AccRotated = zeros(sampleSize, 3);

% There are unique values for each row.

% Should test later on to see if the drift is significant.
theta = .58078;     % Pitch correction. Gives 0 for x value
theta2 = -0.07325;   % Roll correction. Gives 0 for y value.
...theta = 0.5790;  % Pitch correction value on spec sheet
...theta2 = -0.0875; % Roll correction value on spec sheet

% 2D Rotation matrices from Wikipedia
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
R2 = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];
    
% Rotate all the values in Acc and put them in AccRotated
% Can probably just change Acc eventually

% AccRotated = Acc*rotm^-1

for t = 1:length(AccRotated)
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

orientation = zeros(size(AccRotated));

for t = 2:length(AccRotated)
    orientation(t,:) = orientation(t-1,:) + (Gyr(t,:) * period);
end