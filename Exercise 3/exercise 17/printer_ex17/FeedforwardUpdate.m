%FEEDFORWARDUPDATE   Compute feedforward signal for next trial.
function f_jplus1 = FeedforwardUpdate(t,r,e_j,u_j,f_j,i)
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

exercise15 = false;
exercise17 = true;

if (exercise15)
    tic
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
    n=3;
    e_j = [e_j(n+1:end); zeros(n,1)];
    alpha = 1;
    [EL,t] = lsim(alpha*Lc,e_j,t);
    % [EL,t] = lsim(alpha*L,e_j,t);
    f_j = EL +f_j;
    [f_jplus1, ~] = lsim(Qjob,f_j,t);
    toc
end

if (exercise17)
   % Containing matrices J, Q and L
   tic
   switch i
       case 1 
%            load('liftedILC17d.mat')
            load('liftedILC17d_Tuned.mat')
   % Changed We
       case 2 
           load('liftedILC17d_we5_wf-3_wdf-1.mat')
       case 3 
           load('liftedILC17d_we1_wf-3_wdf-1.mat')
%             load('liftedILC17d_we2_wf-3_wdf-1.mat')
   % Changed Wf
       case 4 
           load('liftedILC17d_we3_wf-5_wdf-1.mat')
       case 5 
           load('liftedILC17d_we3_wf-1_wdf-1.mat')
   % Changed Wdf
       case 6 
           load('liftedILC17d_we3_wf-3_wdf-2.mat')
       case 7
           load('liftedILC17d_we3_wf-3_wdf0.mat')
       otherwise
           return;
   end
   %
   

   % Compute f_jplus1
   f_jplus1 = Q2*(f_j + L2*e_j);    
   toc
end

% Make sure the feedforward is a column.
f_jplus1 = f_jplus1(:);

% f_jplus1 = [f_jplus1(3+1:end)' zeros(3,1)']';


