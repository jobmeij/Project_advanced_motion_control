%% Advanced motion control exercise 9
clear all; close all; clc

% Load data
load('avis.mat')

%% Bode of data
figure()
bode(G_mod)
grid on
title('Bode G_m_o_d')

figure()
bode(G_frf)
grid on
title('Bode G_f_r_f')

figure()
bode(G_mod)
hold on
bode(G_frf)
hold off
grid on
legend('Bode G_m_o_d','Bode G_f_r_f')


%% 9a
close all
% Determining transfer function from data
np = 4;     % is sufficient
G_est = tfest(G_mod,np)

bOptions = bodeoptions;
bOptions.FreqUnits = 'Hz';

figure()
bode(G_est,bOptions)
hold on
bode(G_mod,bOptions)
bode(G_frf,bOptions)
hold off
grid on
title('Estimated vs. measured G')
legend('G_e_s_t','G_m_o_d','G_f_r_f','Location','Best')
xlim([G_mod.Frequency(2)/(2*pi) G_mod.Frequency(end)/(2*pi)])

% poles
poles = tf([1],[1 0 0]);
pole2 = tf([1],[1/(20*2*pi) 1])
zero = tf([1 10],[1]);
fnotch = 1.52 %*2*pi;
beta1 = 0.009;
beta2 = 0.9 %10e5;
notch = tf([(1/(2*pi*fnotch)^2) (2*beta1)/(2*pi*fnotch) 1],[(1/(2*pi*fnotch)^2) (2*beta2)/(2*pi*fnotch) 1]);
% K = 2*poles*zero*notch*pole2;

pole = tf([1],[1 2*2*pi]);
zero = tf([1/(2.3*2*pi) 1],[1]);
% K = 1*pole*zero*tf([1 0.01*pi],[1 8*pi]);        % Controller for 9c
% K = 1*pole*zero;                                    % Controller for 9a
K = 1*pole*zero*tf([1 8*pi],[1 50*pi]);           % Controller for 9e
    
L = G_est*K;        % Open loop
T = L/(1+L);        % Closed loop with negative feedback
S = 1/(1+L);        % Sensitivity

% Plot open loop
OpenLoop = figure()
bode(L,1:1000,bOptions)
grid on
title('Open-loop bode')

% Plot closed loop
ClosedLoop = figure()
bode(T,1:1000,bOptions)
grid on
title('Closed-loop bode')

% Plot sensitivity
Sensitivity = figure()
bode(S,1:1000,bOptions)
grid on
title('Sensitivity')

% Plot Nyquist
Nyquist = figure()
nyquist(L)

% Plot sensitivity of G_frf
L_frf = zeros(length(G_frf.Frequency),1);
T_frf = zeros(length(G_frf.Frequency),1);
S_frf = zeros(length(G_frf.Frequency),1);
for i = 1:length(G_frf.Frequency)
    f = G_frf.Frequency(i);
    L_frf(i) = G_frf.ResponseData(i) * evalfr(K,f);
    T_frf(i) = L_frf(i)*(1+L_frf(i));
    S_frf(i) = 1/(1+L_frf(i));
end

figure()
semilogx(G_frf.Frequency,db(L_frf))
grid on
title('Open loop transfer G_f_r_f')

figure()
semilogx(G_frf.Frequency,db(T_frf))
grid on
title('Closed loop transfer G_f_r_f')

figure()
semilogx(G_frf.Frequency,db(S_frf))
grid on
title('Sensitivity G_f_r_f')


%% 9b: determine whether K stabilizes G_frf by analyzing the stability of (1+E*T_mod)^-1
close all
% Idea: check if K stabilizes the system through Nyquist?

n = length(G_mod.Frequency);
E = zeros(n,1);
T_mod = E;
SensGmod = E;
for i = 1:n
    f = G_mod.Frequency(i);
    E(i) = (G_frf.ResponseData(:,:,i) - G_mod.ResponseData(:,:,i)) * inv(G_mod.ResponseData(:,:,i));
    T_mod(i) = (G_mod.ResponseData(:,:,i) * evalfr(K,f)) / (1 + G_mod.ResponseData(:,:,i) * evalfr(K,f));
    
    % Compute (1+G_mod * K)^-1
    SensGmod(i) = (1 + G_mod.ResponseData(:,:,i) * evalfr(K,f)).^-1;
end

% Compute delta sensitivity
SensErr = (1+E.*T_mod).^-1;

% Compute sensitivity G_frf
SensGfrf = SensErr .* SensGmod;     % = (1+G_frf*K)^-1

% Now (1+E*T_mod)^-1 is computed, how to define appropriate bounds on E and
% T_mod? 
for i = 1:n
    f = G_mod.Frequency(i);
    [U,S,V] = svd(T_mod(i));
    
    Sval(i,1) = f;
    Sval(i,2) = S;
end

% compute sigma(T) and mu(E)^-1
for i = 1:length(E)
    M(i) = det(1+E(i).*T_mod(i)); % Computed but not used
    [uval,sval,vval] = svd(M(i));           % do SVD on M to compute delta
    delta(i) = (1/sval)*vval*conj(uval);    % Compute delta
%     test(i) = det(1-M(i).*delta(i));        % Test if singular: it is.
    
    [~,sigmaT(i),~] = svd(T_mod(i));        % Bound of T
    
    [~,sigmaE,~] = svd(E(i));
%     mu(i) = 1/max(sigmaE);
    mu(i) = (1/(abs(E(i))));                % Bound of E (according to chapter 8.8 of textbook)
%     mu(i) = abs(E(i));
end

% plot sigma(T) and mu(E)^-1
figure()
hold on
plot(G_mod.Frequency,sigmaT)
plot(G_mod.Frequency,mu)
hold off
grid on
title('sigma(T) vs 1/mu(E)')
legend('sigma(T)','1/mu(E)')
xlabel('Frequency [Hz]')
ylabel('Amplitude [-]')
xlim([G_mod.Frequency(1) G_mod.Frequency(end)])

figure()
plot(real(M),imag(M))
grid on
title('determinant of (1+E*T_mod)')

if (true)
    % Plotting
    figure()
    plot(real(SensGfrf),imag(SensGfrf))
    grid on
    title('SensGfrf')
    xlabel('real')
    ylabel('imag')
    
    % E
    figure()
    subplot 211
    semilogx(G_mod.Frequency,db(E))
    grid on
    title('E')
    ylabel('Gain [dB]')
    subplot 212
    semilogx(G_mod.Frequency,angle(E)*(180/pi))
    grid on
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
    
    % T_mod
    figure()
    subplot 211
    semilogx(G_mod.Frequency,db(T_mod))
    grid on
    title('T_m_o_d')
    ylabel('Gain [dB]')
    subplot 212
    semilogx(G_mod.Frequency,angle(T_mod)*(180/pi))
    grid on
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
    
    % (1+E*T_mod)^-1
    figure()
    subplot 211
    semilogx(G_mod.Frequency,db(SensErr))
    grid on
    title('1/(1+E*T_m_o_d)')
    ylabel('Gain [dB]')
    subplot 212
    semilogx(G_mod.Frequency,angle(SensErr)*(180/pi))
    grid on
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
    
    % G_frf sensitivity computed from error
    figure()
    subplot 211
    semilogx(G_mod.Frequency,db(abs(SensGfrf)))
    grid on
    title('Bode of G_f_r_f sensitivity computed from E and T_m_o_d')
    ylabel('Gain [dB]')
    subplot 212
    semilogx(G_mod.Frequency,angle(SensGfrf)*(180/pi))
    grid on
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
    
    % SensErr vs SensGfrf
    figure()
    subplot 211
    semilogx(G_mod.Frequency,db(abs(SensGfrf)))
    hold on
    semilogx(G_mod.Frequency,db(SensErr))
    hold off
    grid on
    title('SensErr vs. G_f_r_f sensitivity computed from E and T_m_o_d')
    ylabel('Gain [dB]')
    subplot 212
    semilogx(G_mod.Frequency,angle(SensGfrf)*(180/pi))
    hold on
    semilogx(G_mod.Frequency,angle(SensErr)*(180/pi))
    hold off
    grid on
    legend('SensErr','G_f_r_f sens','Location','Best')
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
end
