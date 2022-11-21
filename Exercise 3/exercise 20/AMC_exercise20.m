%% Advanced motion control - set 3 exercise 20
clear all; close all; clc;

% Given system
Jz.A = 0.5;
Jz.B = 1;
Jz.C = 1;
Jz.D = 1;
Jz.Ts = 1;
[num,den] = ss2tf(Jz.A,Jz.B,Jz.C,Jz.D,Jz.Ts)
Gtf = tf(num,den,Jz.Ts)

Q = 1;
L = 1;  % alpha
C = 1;

J = 1;

% 1. Determining for which value of L the system is convergent
% Determine e(inf)
r=1;
[conv, monConv, e_inv_xr] = liftedIlcConv(Q,L,J)

% 2. 
Q2 = 1*eye(2);
L2 = 0.9*eye(2);  % alpha
%
N2 = 2;
[Y2] = dimpulse(Gtf.num,Gtf.den,N2);
r2 = zeros(N2,1);
J2 = toeplitz(Y2,r2);
%
[conv2, monConv2, e_inv_xr2] = liftedIlcConv(Q2,L2,J2)

figure()
stairs(1:length(Y2),Y2)
grid on

% 3. 
N3 = 3;
Q3 = 1*eye(N3);
L3 = 0.75*eye(N3);  % alpha
%
[Y3] = dimpulse(Gtf.num,Gtf.den,N3);
r3 = zeros(N3,1);
J3 = toeplitz(Y3,r3);
%
[conv3, monConv3, e_inv_xr3] = liftedIlcConv(Q3,L3,J3)

% 4.




%
%
% Function for computing convergence
function [conv, monConv, e_inf_xr] = liftedIlcConv(Q,L,J)
% Convergence: maximum eigenvalue of (Q-L*J) < 1
conv = abs(max(eig(Q-L*J)));

% Monotonic convergence: in 2 norm obtained if maxSigma(Q-L*J) < 1
[U,S,V] = svd(Q-L*J);
maxSigma = max(diag(S));
monConv = norm(maxSigma*(Q-L*J),2); % computing 2 norm, should be below 1

e_inf_xr = (1-J*inv(1-Q+L*J)*L)
end

