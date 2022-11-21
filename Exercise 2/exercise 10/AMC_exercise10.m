%% AMC Exercise 10 
clear all; close all; clc

% Load file
load('wafer.mat')

bOptions = bodeoptions;
bOptions.FreqUnits = 'Hz';

%% Plotting of data
figure()
bode(G_d,bOptions)
grid on
title('G_d')

figure()
bode(G_frf,bOptions)
grid on
title('G_frf')

figure()
bode(G_nd,bOptions)
grid on
title('G_nd')

%% a) creating a decenteralized controller based on G_d
close all; clc;

% Determine gains
xyzGain = 10^(38.5/20) * 10^(80/20);    % Gain for x, y and z axis
rGain = 10^(80/20);                     % Gain for rotation axis

% Lead filter for extra phase (x,y and z axis)
fc = 100;            % Desired cross-over frequency
f1 = 33;            % Start frequency of lead
f2 = (fc^2)/f1      % Computed stop frequency of lead (from start and cross-over freq)
leadFilt = tf([(1/(2*pi*f1)) 1],[(1/(2*pi*f2)) 1])

leadFiltArray = zeros(length(G_d.Frequency),1);
for f = 1:length(G_d.Frequency)
    freq = G_d.Frequency(f);
    leadFiltArray(f) = evalfr(leadFilt,freq);
end

% leadFiltFrd = frd(leadFilt,G_d.Frequency);

% Estimate transfer function of G_d
np =   [4 0 0 0 0 0;
        0 4 0 0 0 0;
        0 0 4 0 0 0;
        0 0 0 4 0 0;
        0 0 0 0 4 0;
        0 0 0 0 0 4];
G_d_est = tfest(G_d,np);

% Create controller
K = tf([1],[1])*eye(6);
K(1,1) = xyzGain * leadFilt * 10^(-9/20);     % x axis
K(2,2) = xyzGain  * leadFilt * 10^(-9/20);    % y axis
K(3,3) = 10^(9.24/20);              % Rz axis
K(4,4) = xyzGain  * leadFilt * 10^(-9/20);    % z axis
K(5,5) = 10^(2.79/20);              % Rx axis
K(6,6) = 10^(3.2/20);               % Ry axis

% Compute several frequency responses
% L = K*G_d;          % Open loop
L = K*G_d_est;
T = L/(1+L);        % Close loop
S = 1/(1+L);        % Sensitivity

% Plot open loop
figure()
bode(L,bOptions)
grid on
title('Open loop')

% Plot closed loop
figure()
bode(T,bOptions)
grid on
title('Closed loop')

% Plot sensitivity
figure()
bode(S,bOptions)
grid on
title('Sensitivity')

% Determine the maximum frequency fbb




