%FEEDFORWARDUPDATE   Compute feedforward signal for next trial.
function f_jplus1 = FeedforwardUpdate(t,r,e_j,u_j,f_j)
Fcausal = true;
Fnoncausal = false;

%OUT:
%f_jplus1 = feed forward signal for upcoming trial
%
%IN:
%t = trial time vector
%r = current trial position reference
%e_j = current trial tracking error
%u_j = current trial control effort
%f_j = current trial feed forward signal

% Load trajectory for feedforward exercise.
% load('trajectory.mat');

% Load Learning controller (uncomment for ILC implementation).
% load('ILCController.mat');

% Calculate feedforward.
% f_jplus1 = 0*t;

% Determine feedforward for exercise 15b
Ts = 0.001;
C_ff = 0.1;


%
MovingAvgLength = 20;
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
f_f = C_ff * r_ddot;

f_jplus1 = f_f;

%% Exercise 15f:
% Determining Q:
fc = 100;
fs = 1/Ts;
order = 6;
[b,a] = butter(order,fc/(fs/2));
butterTf = tf(b,a,Ts);
Q = 0.68 * butterTf;
% Determining L:
numL = [565673.249079546,-2950216.86934670,6384808.10042945,-7234821.44164365,4346944.99020965,-1099839.05205704,-84041.0830831588,71500.4259437121,1.94800741489507,0,0,0];
denL = [0,0,0,1,-1.37023569134266,0.172283824550191,0.310441941553110,-0.0483179546703031,0,0,0,0];
L = tf(numL,denL,Ts);
% Output f_j+1

if (Fcausal)
    numLc = [565673.249079546,-2950216.86934670,6384808.10042945,-7234821.44164365,4346944.99020965,-1099839.05205704,-84041.0830831588,71500.4259437121,1.94800741489507];
    denLc = [1,-1.37023569134266,0.172283824550191,0.310441941553110,-0.0483179546703031,0,0,0,0];
    Lc = tf(numLc,denLc,Ts);
    
    % e_j(end,1);
    % Leval = lsim(L,[0, e_j(end,1)],[0, Ts])
    Lceval = lsim(Lc,[0 e_j(end,1)],[0 Ts]);
    Lceval = Lceval(end);
    
    A = [0, (f_j(end)+Lceval)];
    Qeval = lsim(Q,A,[0, Ts]);
    Qeval = Qeval(end);
    
    % f_jplus1 = lsim(Q,(f_j+lsim(L,e_j,Ts),Ts);
    
    f_jplus1(end) = Qeval;
end

% Make sure the feedforward is a column.
f_jplus1 = f_jplus1(:);