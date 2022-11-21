%% Advanced motion control - exercise 1.2
clear all; close all; clc;

% Insert plant and Lambda matrix
G = [16.8 30.5 4.3; -16.7 31.0 -1.41; 1.27 54.1 5.40];
L = [1.5 0.99 -1.48; -0.41 0.97 0.45; -0.08 -0.95 2.03];

%
RGA = G.*inv(G).'

% Compute sum norm
RGAnumber = norm((L-eye(length(L))),1);

