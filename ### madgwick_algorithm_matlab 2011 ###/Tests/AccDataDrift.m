%close all;
%clear;
%clc;

addpath('../quaternion_library');
addpath('..');
load('../straight walk, 1000 steps.mat');

% Tests how far the acceleration data drifts from it's original readings by
% calculating its mean at different intervals. Since the raw data has
% gravity imbedded, the only way I can think of to accurately test its 
% deviance from the average is to calculate its magnitude THEN subtract
% gravity from the scalar. In this way, the average magnitude of the raw
% data should be 0. -- DOES NOT WORK, MAGNITUDE IS ALWAYS POSITIVE

% The goal of this test is to give a benchmark for how much drift is to be
% expected in the final velocity and position results.

%% Settings
period = 1/100;
beta = .1;
sampleSize = 122002;
pitchCorrection = 0.0875;
rollCorrection = 0.5790;
yawCorrection = 0;
gravity = 9.786;

%% Initialization

sums = zeros(100,3);
%% Function

for step = 1:100
    first = 900 + (step*120); % 120 is step size
    last = first + 82; % 82 is length of step
    sums(step,1) = sum(AbsAcc(first:last,1));
    sums(step,2) = sum(AbsAcc(first:last,2));
    sums(step,3) = sum(AbsAcc(first:last,3));
end

disp(sums);


figure;
plot(1:100, sums(:,1), 'r');