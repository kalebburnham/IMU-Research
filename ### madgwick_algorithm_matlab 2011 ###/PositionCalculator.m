close all;
clc;
clear;

load('ExampleData.mat');

acc = zeros(length(time), 3);

%% Compute Absolute Accelerations

period = 1/256;
AHRS = MadgwickAHRS('SamplePeriod', period);

% Clean magnetometer readings

for t = 1:length(time)
    AHRS.Update(Gyroscope(t, :) * (pi/180), Accelerometer(t, :), Magnetometer(t, :));
    quat = AHRS.Quaternion;
    rotMat = quatern2rotMat(quat);
    
    % Matrix right division. Same as Accelerometer(t,:)' / rotMat.
    % This is faster than inv(rotMat) / Accelerometer(t,:)' and equivalent
    acc(t,:) = rotMat \ Accelerometer(t,:)';
    acc(t,3) = acc(t,3)-1; % Remove gravity. Device at rest is now [0 0 0] ?????

    % Apply magnetic declination offset here if necessary.
    
    acc(t,:) = acc(t,:) * 9.80655; % convert from g's to m/s/s
end

%% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
   vel(t,:) = vel(t-1,:) + acc(t,:) * period;
end

%% Integrate velocity to yield positions
pos = zeros(size(vel));
for t = 2:length(pos)
   pos(t,:) = pos(t-1,:) + vel(t,:) * period; 
end

%% Plot Position
figure('Name', 'Position');
plot3(pos(:,1), pos(:,2), pos(:,3));

%% Plot Accelerations
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');
