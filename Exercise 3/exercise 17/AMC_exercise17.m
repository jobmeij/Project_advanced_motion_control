%% Advanced motion control - exercise set 3 - exercise 17
% Lifted ILC applied to the printer system
clear all; close all; clc;

% Add printer setup files
addpath('C:\Users\Job\Google Drive\S&C master\Tom en Job\4CM60 Advanced Motion Control\Exercise 3\exercise 17\printer_ex17')

% Load the FRF, parametric model G and controller C
load('PrinterModel.mat')        % Importing FRF and G
load('PrinterController.mat')   % Importing C

% Determine transfer function of G*S using minreal
Ts = PrinterModelSS.Ts;
Gtf = tf(PrinterModelSS);
GS = minreal(feedback(Gtf,shapeit_data.C_tf_z));

%% 17a)
% A time domain model is required now in the lifted domain.
% Setting trial length to 100 and construct J (using dimpulse and toeplitz)

% Creating impulse response of GS
Nlength = 100;
Tend = (Nlength-1)*GS.Ts;
[Y,t] = dimpulse(GS.Numerator,GS.Denominator,Nlength);

% Creating Toeplitz matrix from impulse response (discrete time
% convolution)
r = zeros(1,length(Y))';
J = toeplitz(Y,r);
% J = round(J,12); % Setting small insertions to 0 to ease computations

plot17a = true;
if (plot17a)
    figure()
    hold on
    stairs(1:length(Y),Y)
    hold off
    grid on
    title('Impulse response of GS')
    xlabel('Sample [N]')
    ylabel('Amplitude [-]')
    
    figure()
    imagesc(J)
    title('Image of Toeplitz matrix J','interpreter','latex')
    colorbar
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    
    figure()
    mesh(J)
    title('Mesh of Toeplitz matrix J','interpreter','latex')
    colorbar
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
end

%% 17b) Designing a lifted ILC controller
we = 10^3;          % was 10^3
wf = 10^-3;         % was 10^-3
wdf = 10^-1;        % was 10^-1

% Compute L and Q matrices
[L,Q] = computeLQ(we,wf,wdf,J);

% Computing what happens when wf is 0
[L0,Q0] = computeLQ(we,0,wdf,J);

% Compute delta
L0d = L-L0;
Q0d = Q-Q0;

% Plotting L and Q
if (true)
    i = 0;    % iterator for selecting the diagonal from the original diagonal
    figure()
    hold on
    plot(diag(L,i))
    plot(diag(Q,i))
    hold off
    grid on
    title('Diagonal of L and Q','interpreter','latex')
    legend('diag L','diag Q','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    
    figure()
    mesh(L)
    title('Matrix L with N=100','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    colorbar
    
    figure()
    imagesc(L)
    title('Matrix L with N=100','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    colorbar
    
    figure()
    mesh(Q)
    title('Matrix Q with N=100','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    colorbar
    
    figure()
    imagesc(Q)
    title('Matrix Q with N=100','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    colorbar    
    
    figure()
    imagesc(L0d)
    title('Delta between L with wf=0.001 and 0','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    colorbar    
    
    figure()
    imagesc(Q0d)    
    title('Delta between Q with wf=0.001 and 0','interpreter','latex')
    xlabel('Row','interpreter','latex')
    ylabel('Column','interpreter','latex')
    colorbar    
end

% - what is the structure?
% - how does this relate to causality and time (in)variance?
% - what happens to L and Q if wf is set to zero? (vary with we, wf and wdf)
% - how should the values be chosen to guarantee lim(e)inf = 0? (monotonic
%   convergence: max singular value of (Q-LJ) < 1

% 17c) Investigate convergence and monotonic convergence properties of the
%   lifted ILC controller
[conv, monConv] = liftedIlcConv(Q,L,J)
[conv0, monConv0] = liftedIlcConv(Q0,L0,J)

% Compute steady state error with given reference
r = 10;
detLJ = det(L*J)        % Check singularity
errorSS = steadyStateError(L,Q,J,r);

%% 17d) Implement and simulate the lifted ILC controller (using N=4501)
% while investigating the finluence of we, wf and wdf on convergence speed
% and ILC performance
Nlength2 = 4501;
% Tend2 = (Nlength2-1)*GS.Ts;
% [Y2,t2] = impulse(GS,Tend2);
[Y2,t2] = dimpulse(GS.Numerator,GS.Denominator,Nlength2);

% Creating Toeplitz matrix from impulse response
r2 = zeros(1,length(Y2));
J2 = toeplitz(Y2,r2);
% J2 = round(J2,12);   % Setting small insertions to 0 to ease computations

% Computing new L and Q matrices
we2 = 10^5;         % was 10^3, set to 1 and 5 for determining value
wf2 = 10^(-5);        % was 10^-3, set to -1 and -5 for determining value
wdf2 = 10^(-2);       % was 10^-1, set to 0 and -2 for determining value

% Compute L and Q matrices
[L2,Q2] = computeLQ(we2,wf2,wdf2,J2);

% Check convergence
[conv2, monConv2] = liftedIlcConv(Q2,L2,J2)

% Plotting
plot17d = true;
if (plot17d)
    % Impulse response plot
%     figure()
%     hold on
%     stairs(1:length(Y2),Y2)
%     hold off
%     grid on
%     title('Impulse response of GS with N=4501')
%     xlabel('Time [s]')
%     ylabel('Amplitude [-]')
    
    % Toeplitz matrix imagesc
%     figure()
%     imagesc(J2)
%     title('Image of Toeplitz matrix J with N=4501','interpreter','latex')
%     colorbar
%     xlabel('Row','interpreter','latex')
%     ylabel('Column','interpreter','latex')
%     
%     % Toeplitz matrix mesh
%     figure()
%     mesh(J2)
%     title('Mesh of Toeplitz matrix J with N=4501','interpreter','latex')
%     colorbar
%     xlabel('Row','interpreter','latex')
%     ylabel('Column','interpreter','latex')
    
%     figure()
%     mesh(L2)
%     title('Matrix L with N=4501','interpreter','latex')
%     xlabel('Row','interpreter','latex')
%     ylabel('Column','interpreter','latex')
%     colorbar
    
%     figure()
%     imagesc(L2)
%     title('Matrix L with N=4501','interpreter','latex')
%     xlabel('Row','interpreter','latex')
%     ylabel('Column','interpreter','latex')
%     colorbar
    
%     figure()
%     mesh(Q2)
%     title('Matrix Q with N=4501','interpreter','latex')
%     xlabel('Row','interpreter','latex')
%     ylabel('Column','interpreter','latex')
%     colorbar
    
%     figure()
%     imagesc(Q2)
%     title('Matrix Q with N=4501','interpreter','latex')
%     xlabel('Row','interpreter','latex')
%     ylabel('Column','interpreter','latex')
%     colorbar    
end

%% 17e) Comparing the lifted ILC with the frequency domain ILC controller




%% Functions
%
%
% Function for computing the L and Q matrices
function [L,Q] = computeLQ(we,wf,wdf,J)
% Determining weighting matrices
Nlength = length(J);
We = we*eye(Nlength);
Wf = wf*eye(Nlength);
Wdf = wdf*eye(Nlength);

% Computing L and Q
% X = inv(J'*We*J+Wf+Wdf);
X = (J'*We*J+Wf+Wdf);
Q = X\(J'*We*J+Wdf);
L = X\(J'*We);
end

%
%
% Function for computing convergence
function [conv, monConv] = liftedIlcConv(Q,L,J)
% Convergence: maximum eigenvalue of (Q-L*J) < 1
conv = abs(max(eig(Q-L*J)));

% Monotonic convergence: in 2 norm obtained if maxSigma(Q-L*J) < 1
[U,S,V] = svd(Q-L*J);
maxSigma = max(diag(S));
monConv = norm(maxSigma*(Q-L*J),2); % computing 2 norm, should be below 1
end

function [errorSS] = steadyStateError(L,Q,J,r)
N = length(L);
errorSS = (eye(N)-J*inv(eye(N)-Q+L*J)*L)*r;
end

