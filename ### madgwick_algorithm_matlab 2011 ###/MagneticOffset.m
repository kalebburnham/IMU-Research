close all;
clear;
clc;
addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

%% Plot MagX and MagY

figure;
scatter(Mag(:,1), Mag(:,2), 'r');
figure;
plot(Mag(:,2), Mag(:,3), 'r');