close all;
clc;
clear;

addpath('quaternion_library');
load('straight walk, 1000 steps.mat');

H = [-1 0 0; 
      1 0 0];