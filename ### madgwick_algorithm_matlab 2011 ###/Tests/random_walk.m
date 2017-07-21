close all;
clc;
clear;

size = 1000000;
totalNoise = zeros(size,1);


% Add white noise. Acc = 0.012 m/s/s, Gyr = 0.0087 rad/s

for t = 2:size
    mean = 0;
    sigma = 0.012;
    
    n = normrnd(mean, sigma); % Acceleration noise
    totalNoise(t,:) = totalNoise(t-1,:) + n;
end

plot(1:size, totalNoise(:,1));