%SIMILC   Iterative Learning Control simulations.
clear variables; close all; clc;

% Number of trials.
N_trial = 25;

% Load trajectory.
load('trajectory.mat');
t = t_ref;
r = x_ref;
N = length(r);

% Initial feedforward.
f_jplus1 = zeros(N,1);

% Initialize plotting.
PlotTrialData;

% Initialize storage variables.
history.f     = NaN(N,N_trial);
history.u     = NaN(N,N_trial);
history.e     = NaN(N,N_trial);
history.eNorm = NaN(1,N_trial);

%%%%%%%%%%%%%
% Start ILC %
%%%%%%%%%%%%%
for trial = 1:N_trial
    % Update index.
    f_j = f_jplus1;
    
    % Store feedforward and reference for use in simulation.
    loaddata = [f_j, r];
    fp = fopen('loaddata.dat','w');
    fprintf(fp,'%25.18f\t%25.18f\n',loaddata.');
    fclose(fp);
    
    % Start simulation.
    sim('PrinterILC_sim');
    
    % Display trial number.
    fprintf('Experiment nr. %d/%d finished.\n',trial,N_trial);
    
    % Load and extract trial data.
    load('expdata.dat');
    t   = expdata(:,1);
    r   = expdata(:,2);
    e_j = expdata(:,3);
    u_j = expdata(:,4);
    
    % Determine feedforward for exercise 15b
    Ts = 0.001;
    C_ff = 1;
    %
    MovingAvgLength = 25;
    %
    r_dot = [0; ((r(2:end) - r(1:(end-1),1))/(Ts))];
    %
    A_dot = 1;
    B_dot = (1/MovingAvgLength)*ones(MovingAvgLength,1);
    r_dot = filtfilt(B_dot,A_dot,r_dot);
    %
    r_ddot = [0; ((r_dot(2:end) - r_dot(1:(end-1),1))/(Ts))];
    %
    A_ddot = A_dot;
    B_ddot = B_dot;
    r_ddot = filtfilt(B_ddot,A_ddot,r_ddot);
    %
    
    % Store trial data.
    history.f(:,trial)     = f_j;
    history.u(:,trial)     = u_j;
    history.e(:,trial)     = e_j;
    history.eNorm(:,trial) = norm(e_j,2);
    
    % Calculate new feedforward.
    f_jplus1 = FeedforwardUpdate(t,r,e_j,u_j,f_j);
    
    % Plot results.
    PlotTrialData;
    
end

% End.
disp('Done!');

%
figure()
hold on
plot(r)
plot(r_dot)
plot(r_ddot)
hold off
grid on
legend('r','rd','rdd')
