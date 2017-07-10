clc;
clear;
close all;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat'); % 122002 samples
...load('Closed trajectory, 1 loop.mat'); % 11616 samples
...load('Closed trajectory, 10 loops.mat'); % 98160 samples

%% Settings
period = 1/100;
sampleSize = 5000;
beta = 0.08;
gravity = 9.786;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;

integrationMethod = 'Midpoint'; % Midpoint or Trapezoidal

%% Initialization

% Find initial quaternion and create AHRS
rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

% Create variables
quaternion = zeros(sampleSize, 4);
AbsAcc = zeros(sampleSize, 3); % Acceleration with respect to global axes - 'Absolute Acceleration'
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);

%% Function

% Find acceleration in global frame
for t = 1:sampleSize
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    quaternion(t,:) = AHRS.Quaternion;
    
    % Perform weird adjustment
    quaternion(t,2:4) = -1 * quaternion(t,2:4);
    
    % Rotate data to match global reference frame
    AbsAcc(t,:) = Acc(t,:) * inv(quatern2rotMat(quaternion(t,:)));
        
    % Subtract Gravity
    AbsAcc(t,3) = AbsAcc(t,3) - gravity;
end


% Integrate for velocity and position
if strcmp(integrationMethod, 'Midpoint')
    for t = 1000:sampleSize
        vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
        pos(t,:) = pos(t-1,:) + (vel(t,:) * period);    % vel -> pos
    end
elseif strcmp(integrationMethod, 'Trapezoidal')
    for t = 1000:sampleSize
        vel(t,:) = vel(t-1,:) + [AbsAcc(t-1,:) + AbsAcc(t,:)] * 0.5 * period; % Acc -> vel
        pos(t,:) = pos(t-1,:) + [vel(t-1,:) + vel(t,:)] * 0.5 * period;       % vel -> pos
    end
end

%% Plot results

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, pos(1:sampleSize,1), 'r');
plot(1:sampleSize, Pos(1:sampleSize,1), 'g');
title('Position Estimate on X-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, pos(1:sampleSize,2), 'r');
plot(1:sampleSize, Pos(1:sampleSize,2), 'g');
title('Position Estimate on Y-axis');
legend('Estimate', 'Ground Truth');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, pos(1:sampleSize,3), 'r');
plot(1:sampleSize, Pos(1:sampleSize,3), 'g');
title('Position Estimate on Z-axis');
legend('Estimate', 'Ground Truth');
hold off;

figure;
axis(1) = subplot(2,1,1);
x = pos(1:sampleSize,1);
y = pos(1:sampleSize,2);
z = pos(1:sampleSize,3);
plot3(x,y,z);
title('Position Estimate');
axis(2) = subplot(2,1,2);
X = Pos(1:sampleSize,1);
Y = Pos(1:sampleSize,2);
Z = Pos(1:sampleSize,3);
plot3(X,Y,Z);
title('Position Ground Truth');
