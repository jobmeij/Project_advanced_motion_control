%FEEDFORWARDUPDATE   Compute feedforward signal for next trial.
function f_jplus1 = FeedforwardUpdate(t,r,e_j,u_j,f_j)
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
% 
% C_ff = 0.1;
% f_jplus1 = C_ff * a_ref;

% Determining Q: (ADDED BY JOB)
Ts = 0.001;
fc = 100;
fs = 1/Ts;
order = 6;
[b,a] = butter(order,fc/(fs/2));
butterTf = tf(b,a,Ts);
Qjob = 0.68 * butterTf;
% Qjob = butterTf;
%

% Load Learning controller (uncomment for ILC implementation).
% load('ILCController.mat');
load('ILCdataJob.mat');
e_j = [e_j(5+1:end); zeros(5,1)];
alpha = 1;
[EL,t] = lsim(alpha*Lc,e_j,t);
% [EL,t] = lsim(alpha*L,e_j,t);
f_j = EL +f_j;
[f_jplus1, ~] = lsim(Qjob,f_j,t);
% [f_jplus1, ~] = lsim(Q,f_j,t);

% f_jplus1(:,:) = 0;

% Make sure the feedforward is a column.
f_jplus1 = f_jplus1(:);