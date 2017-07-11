clc;
clear;
close all;
addpath('quaternion_library');

load('straight walk, 1000 steps.mat'); % 122002 samples
...load('Closed trajectory, 1 loop.mat'); % 11616 samples
...load('Closed trajectory, 10 loops.mat'); % 98160 samples

%% Settings
period = 1/100;
sampleSize = 10000; % Must be greater than 1000
beta = 0.08;
gravity = [0 0 -9.786];
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;
first = 1000;

%% Initialization

% Find initial quaternion and create AHRS
rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);

% Create variables
AbsAcc = zeros(sampleSize, 3); % Acceleration with respect to global axes - 'Absolute Acceleration'
vel = zeros(sampleSize, 3); % lower case = test results, not ground truth
pos = zeros(sampleSize, 3);

q = zeros(sampleSize, 4);
grav = zeros(sampleSize,3);

%% Function

% Find acceleration in global frame
for t = 1:sampleSize
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    q(t,:) = AHRS.Quaternion;
    
    grav(t,1) = 2*(q(t,2)*q(t,4) - q(t,1)*q(t,3));
    grav(t,2) = 2*(q(t,1)*q(t,2) + q(t,3)*q(t,4));
    grav(t,3) = q(t,1)*q(t,1) - q(t,2)*q(t,2) - q(t,3)*q(t,3) + q(t,4)*q(t,4);
    
    AbsAcc(t,1) = Acc(t,1) + gravity(3)*grav(t,1);
    AbsAcc(t,2) = Acc(t,2) + gravity(3)*grav(t,2);
    AbsAcc(t,3) = Acc(t,3) + gravity(3)*grav(t,3);
end


% Integrate for velocity and position

for t = first:sampleSize
    vel(t,:) = vel(t-1,:) + (AbsAcc(t,:) * period); % Acc -> vel
    pos(t,:) = pos(t-1,:) + (vel(t,:) * period);    % vel -> pos
end

%% Plot results


%Velocity
figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, vel(1:sampleSize,1), 'r');
plot(1:sampleSize, Vel(1:sampleSize,1), 'g');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, vel(1:sampleSize,2), 'r');
plot(1:sampleSize, Vel(1:sampleSize,2), 'g');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, vel(1:sampleSize,3), 'r');
plot(1:sampleSize, Vel(1:sampleSize,3), 'g');
hold off;

% 2D results
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

% 3D Position
figure;
axis(1) = subplot(2,1,1);
plot3(pos(1:sampleSize,1), pos(1:sampleSize,2), pos(1:sampleSize,3), 'r');
title('Position Estimate');
axis(2) = subplot(2,1,2);
plot3(Pos(1:sampleSize,1), Pos(1:sampleSize,2), Pos(1:sampleSize,3), 'r');
title('Position Ground Truth');