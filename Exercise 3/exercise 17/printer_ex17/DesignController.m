%DESIGNCONTROLLER   Use this template for designing your controller.

clear variables;
close all;
clc;


%% Parameters
fs = 1000;         % sample frequency
Ts = 1/fs;         % sample time
N = 4501;          % trial length
save_switch = 1;   % switch for saving ILC filters: 1 = save, 0 = no save


%% System model, system frequency response measurement and controller.
% System model and FRF measurement.
load('PrinterModel.mat');
G = PrinterModelSS;
Gfrf = PrinterModelFRF;

% Feedback controller.
load('PrinterController.mat');
C = minreal(shapeit_data.C_tf_z);


%% Iterative Learning Controller design.
%YOUR CODE HERE


%% Save ILC filters.
if save_switch
    % Modify line below to choose which variables to save
    save('ILCController','L','Q');    
    disp('Parameters saved!');
end