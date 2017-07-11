clc;
clear;
close all;
addpath('../quaternion_library');
addpath('..');

load('../straight walk, 1000 steps.mat'); % 122002 samples
...load('../Closed trajectory, 1 loop.mat'); % 11616 samples
...load('../Closed trajectory, 10 loops.mat'); % 98160 samples

%% Settings
period = 1/100;
sampleSize = 10000; % Must be greater than 1000
beta = 0.08;
gravity = [0 0 -9.786];
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;
first = 1000;

%% Test

% Find initial quaternion and create AHRS
rotm = euler2rotMat(pitchCorrection, rollCorrection, yawCorrection);
Quaternion = rotMat2quatern(rotm);
AHRS = MadgwickAHRS('SamplePeriod', period, 'Beta', beta, 'Quaternion', Quaternion);


q = zeros(sampleSize, 4);
grav = zeros(sampleSize,3);
vel = zeros(sampleSize, 3);

AbsAcc1 = zeros(sampleSize, 3);
AbsAcc2 = zeros(sampleSize, 3);
AbsAcc3 = zeros(sampleSize,3);

% Find acceleration in global frame
for t = 1:sampleSize
    % Update quaternion
    AHRS.Update(Gyr(t,:), Acc(t,:), Mag(t,:));
    q(t,:) = AHRS.Quaternion;
    
    % Perform weird adjustment
    q(t,2:4) = -1 * q(t,2:4);
    
    % CLASSICAL METHOD
    % Rotate data to match global reference frame
    AbsAcc1(t,:) = Acc(t,:) * inv(quatern2rotMat(q(t,:)));
    % Subtract Gravity
    AbsAcc1(t,3) = AbsAcc1(t,3) - 9.786;
    
    % NEW METHOD
    grav(t,1) = 2*(q(t,2)*q(t,4) - q(t,1)*q(t,3));
    grav(t,2) = 2*(q(t,1)*q(t,2) + q(t,3)*q(t,4));
    grav(t,3) = q(t,1)*q(t,1) - q(t,2)*q(t,2) - q(t,3)*q(t,3) + q(t,4)*q(t,4);
    
    
    
    AbsAcc2(t,1) = Acc(t,1) - gravity(3)*grav(t,1);
    AbsAcc2(t,2) = Acc(t,2) - gravity(3)*grav(t,2);
    AbsAcc2(t,3) = Acc(t,3) - gravity(3)*grav(t,3);
    
    
    AbsAcc3(t,:) = quatrotate(q(t,:), Acc(t,:)) + gravity;
    
end

for t = first:sampleSize
    vel(t,:) = vel(t-1,:) + (AbsAcc3(t,:) * period); % Acc -> vel
end

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, AbsAcc3(:,1), 'r');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, AbsAcc3(:,2), 'r');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, AbsAcc3(:,3), 'r');
hold off;

figure;
axis(1) = subplot(3,1,1);
hold on;
plot(1:sampleSize, vel(:,1), 'r');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(1:sampleSize, vel(:,2), 'r');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(1:sampleSize, vel(:,3), 'r');
hold off;

