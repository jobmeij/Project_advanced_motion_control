%% AMC exercise set 3 - exercise 18
clear all; close all; clc;

% Declaring variables
J = 1;
N = 1;
L = -0.4;
Q = -0.8;
r = 1;
iterations = 20;

% Determining convergence
[conv, monConv] = liftedIlcConv(Q,L,J)

% Some debugging
% we = -1;
% wf = 2;
% wdf = 0.1;
% [L,Q] = computeLQ(we,wf,wdf,J)

% Determine e(inf)
e_inf = (1-J*inv(1-Q+L*J)*L)*r;


%
%
% Running loop
e_j = zeros(iterations,1);
f_j = zeros(iterations,1);
f_jplus1 = zeros(iterations,1);
for i = 1:iterations    
    
    % Set f_j
    if (i > 1)
        f_j(i) = f_jplus1(i-1);
    else
        f_j(i) = 0;
    end
    
    
    % Compute e_j and f_jplus1
    e_j(i) = r-J*f_j(i);
    f_jplus1(i) = Q*f_j(i) + L*e_j(i); 
    
    e_inf_dif(i) = norm(e_inf - e_j(i),2);
end

%
%
% Plotting
figure()
hold on
plot(1:length(e_j),e_inf_dif,'--x')
hold off
grid on
xlabel('Iteration number','interpreter','latex')
ylabel('$e_{inf}-e_{j}$','interpreter','latex')
title('Steady state error minus current error','interpreter','latex')

figure()
% subplot (2,1,1)
% plot(1:length(e_j),e_j,'--x')
% grid on
% ylabel('$e_{j}$','interpreter','latex')
% title('Signals $e_{j}$, $f_{j}$ and $f_{j+1}$','interpreter','latex') 
% xlim([1 iterations])
% 
% subplot (2,1,2)
hold on
plot(1:length(e_j),e_j,'--x')
plot(1:length(f_j),f_j,'--x')
plot(1:length(f_jplus1),f_jplus1,'--x')
hold off
grid on
xlabel('Iteration number','interpreter','latex')
legend('$e_{j}$','$f_{j}$','$f_{j+1}$','interpreter','latex','Location','Best')
xlim([1 iterations])
title('ILC controller signals','interpreter','latex')

%% Functions
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






