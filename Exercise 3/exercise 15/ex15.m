%% Advanced motion control - exercise 15
clear all; close all; clc;

% Add path of extra files for printer setup
addpath('./printer')

% Load the FRF, parametric model G and controller C
load('PrinterModel.mat')        % Importing FRF and G
load('PrinterController.mat')   % Importing C

% Create transfer function from state space plant model
Ts = PrinterModelSS.Ts;
[NUM,DEN] = ss2tf(PrinterModelSS.A,PrinterModelSS.B,PrinterModelSS.C,PrinterModelSS.D);
G = tf(NUM,DEN,Ts);

C = c2d(shapeit_data.C_tf,Ts);
GC = G*C;                                       % Open-loop discrete time
S = inv(1+GC);                                  % Sensitivity


bOptions = bodeoptions;
bOptions.FreqUnits = 'Hz';

figure(1)
bode(G);
title("plant model")
grid on

% Determine transfer function of G*S using minreal
GS = minreal(feedback(G,C));
n_GS2 = GS.Numerator{1,1};
d_GS2 = GS.Denominator{1,1};

% use zpetc.m to create Lc (causal)
rho = 1; % most of the time equal to 1
[n_Lc, d_Lc, phd] = zpetc(n_GS2,d_GS2,1);
Lc = tf(n_Lc,d_Lc,Ts);       % Resulting in causal L
% Create noncausal L
zd = tf([1, zeros(1,phd)],[1],Ts);
L = zd * Lc;

figure(2)
hold on
bode(L,bOptions)
bode(L*GS,bOptions)
bode(Lc,bOptions)
bode(Lc*GS,bOptions)
legend("L","Lc*GS","Lc","Lc*GS");
grid on

figure(3)
hold on
bodemag((1-L*GS),bOptions)
bodemag((1-L*PrinterModelFRF))
grid on
title("Covergence ILC scheme (1-LGS)")
legend("Model Plant","FRF plant")

% Create Q so 
N = 4;
Wn = 29*2*pi*Ts;  % Hz converted to rad/s and normalized

[n_but,d_but] = butter(N,Wn,'low');
Q = tf(n_but,d_but,Ts);
figure(4)
hold on
bodemag(Q)
bodemag(Q*(1-L*GS),bOptions)
bodemag(Q*(1-L*PrinterModelFRF))
grid on
legend("Q filter","Model Plant","FRF plant")

save('ILCController.mat','Q','Lc')

%%
