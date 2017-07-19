close all;
clear;
clc;

samples = 100000;

noise = normrnd(0,0.012,1000,2);

scatter(noise(:,1), noise(:,2));




function value = noiseCalculator(mean, sigma)
    value = randn();
    %value = normrnd(mean,sigma);
end