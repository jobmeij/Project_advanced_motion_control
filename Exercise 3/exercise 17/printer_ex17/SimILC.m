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
for i = 1:7
    f_jplus1 = zeros(1,4501)';
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
%         Ts = 0.001;
%         C_ff = 1;
        %
%         MovingAvgLength = 25;
        %
%         r_dot = [0; ((r(2:end) - r(1:(end-1),1))/(Ts))];
        %
%         A_dot = 1;
%         B_dot = (1/MovingAvgLength)*ones(MovingAvgLength,1);
%         r_dot = filtfilt(B_dot,A_dot,r_dot);
        
        %
%         r_ddot = [0; ((r_dot(2:end) - r_dot(1:(end-1),1))/(Ts))];
        %
%         A_ddot = A_dot;
%         B_ddot = B_dot;
%         r_ddot = filtfilt(B_ddot,A_ddot,r_ddot);
        %
        
        % Exercise 17a) constructing matrix J
        
        % end 17
        
        % Store trial data.
        history.f(:,trial)     = f_j;
        history.u(:,trial)     = u_j;
        history.e(:,trial)     = e_j;
        history.eNorm(:,trial) = norm(e_j,2);
        
        % Calculate new feedforward.
        f_jplus1 = FeedforwardUpdate(t,r,e_j,u_j,f_j,i);
        
        % Plot results.
        PlotTrialData;
        
    end
    
    eNormData(i,:) = history.eNorm
end

% End.
disp('Done!');


% Plotting error norms of different weighting matrices
figure()
subplot (3,1,1) % Adjusting we
hold on
plot(1:N_trial,eNormData(3,:),'--x')    % we = 10^1
plot(1:N_trial,eNormData(1,:),'--x')    % we = 10^3
plot(1:N_trial,eNormData(2,:),'--x')    % we = 10^5
hold off
grid on
legend('$10^{1}$','$10^{3}$','$10^{5}$','interpreter','latex','Location','Best')
title('Varying $W_{e}$','interpreter','latex')
ylabel('$||e||_{2} [m^{2}]$','interpreter','latex');

subplot (3,1,2) % Adjusting wf
hold on
plot(1:N_trial,eNormData(4,:),'--x')    % wf = 10^-5
plot(1:N_trial,eNormData(1,:),'--x')    % wf = 10^-3
plot(1:N_trial,eNormData(5,:),'--x')    % wf = 10^-1
hold off
grid on
legend('$10^{-5}$','$10^{-3}$','$10^{-1}$','interpreter','latex','Location','Best')
title('Varying $W_{f}$','interpreter','latex')
ylabel('$||e||_{2} [m^{2}]$','interpreter','latex');

subplot (3,1,3) % Adjusting wdf
hold on
plot(1:N_trial,eNormData(6,:),'--x')    % wdf = 10^-6
plot(1:N_trial,eNormData(1,:),'--x')    % wdf = 10^-1
plot(1:N_trial,eNormData(7,:),'--x')    % wdf = 10^0
hold off
grid on
legend('$10^{-2}$','$10^{-1}$','$10^{0}$','interpreter','latex','Location','Best')
title('Varing $W_{\Delta f}$','interpreter','latex')
ylabel('$||e||_{2} [m^{2}]$','interpreter','latex');
xlabel('Trial number','interpreter','latex')

%
% figure()
% hold on
% plot(r)
% plot(r_dot)
% plot(r_ddot)
% hold off
% grid on
% legend('r','rd','rdd')
