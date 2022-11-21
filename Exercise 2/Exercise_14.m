clear all
close all
clc

load hbridge.mat

figure(1)
opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
bodemag(G,opts)
title('Original plant')
grid on

%Translation is measured in m and rotation in rad
% Fy in N and Tz Nm

%Scaling weight
Fbw = 50*2*pi ;%bandwidth HZ
Wsc = 1e-6*[1/(50e-9) 0 ;
            0 1/0.5e-6];
% Vsc = [10^(100/20) 0; 0 10^(85.7/20)];
Vsc = real(freqresp(Wsc*G,Fbw)^-1);

Gtil = Wsc*G*Vsc;


figure(2)
bodemag(Gtil,opts)
title('Scaled plant')
grid on

%% B & C Performance shaping
close all
%parameters
fr1 = 500;
fr2 = 500;
fi1 = 5;
fi2 = 5;
Ks1 = 0.5;
Ks2 = 0.5;
alpha = 20;
s = tf('s');
   
W1 = [Ks1*(s+2*pi*fi1)/s 0;
      0 Ks2*(s+2*pi*fi2)/s];

W2 = [Ks1*(s+2*pi*fr1)/((1/alpha)*s+2*pi*fr1) 0;
      0 Ks2*(s+2*pi*fr2)/((1/alpha)*s+2*pi*fr2)];
figure(3)
bode(W1)

figure(4)
bode(W2)
  
%% H use Hinfsyn
inp = 2;
P_bar = ss(A, [zeros(8,inp), B, B], [-C; zeros(inp,8); -C],[-eye(inp) -D -D; zeros(inp,4), eye(inp); -eye(inp), -D, -D]);

%transform matrices
W = [W1*Wsc, zeros(2,2); 
    zeros(2,2), W2*Vsc^(-1)];

V = [Wsc^(-1), zeros(2,2); 
    zeros(2,2), Vsc];
        
P = [W, zeros(4,2); zeros(2,4), eye(2)]*P_bar*[V, zeros(4,2); zeros(2,4), eye(2)];
nmeas = 2;
ncont = 2;
opts = hinfsynOptions('Display', 'on');
disp('h inf syn:')
[K,CL,gamma] = hinfsyn(P,nmeas,ncont,opts);



%% j shifted poles of W1
close all

epsilon = 1e-3;
%shift poles of W1
P = [W, zeros(4,2); zeros(2,4), eye(2)]*P_bar*[V, zeros(4,2); zeros(2,4), eye(2)];
P.A = P.A - epsilon*eye(size(P.A,1));
opts = hinfsynOptions('Display', 'on');
disp('h inf syn, shifted poles:')
[K,CL,gamma] = hinfsyn(P,nmeas,ncont,opts);

% restore poles:
K.A = K.A + epsilon*eye(size(K.A,1));

%scaled system
K_til = Vsc^-1*K*Wsc^-1;
% K = Vsc*K_til*Wsc;
G_til = Wsc*G*Vsc;
S_til = (eye(2)+G_til*K_til)^(-1);
M_til = [S_til, S_til*G_til; K_til*S_til, K_til*S_til*G_til];



S = (eye(2)+G*K)^(-1);
M = [S, S*G; K*S, K*S*G];

%%
%Create figures
figure(1)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e4];
bodemag(K_til,opts)

figure(2)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e4];
bodemag(K,opts)

figure(3)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1e-2 1e3];
bodemag(CL(1:2,1:2),opts)
hold on 
bodemag((W1)^-1,opts)

figure()
bodemag(G,opts)

figure(3)
bode(W1^-1,opts)

figure(4)
bode(W2^-1,opts)

figure(4)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
bodemag(CL(3:4,3:4),opts)
hold on 
bodemag(W2^-1,opts)




figure(5)
closedloop = lft(P_bar,K_til);
bodemag(closedloop(1:2,1:2),opts)
hold on 
% bodemag((W1)^-1,opts)


figure(6)
closedloop = lft(P_bar,K_til);
bodemag(closedloop(3:4,3:4),opts)
hold on 
% bodemag((W2)^-1,opts)

figure(7)
bodemag(M_til,opts)
hold on
bodemag(Weight,opts)

figure(8)
bodemag(S_til*G_til,opts)
hold on
bodemag((W1*Wsc*Vsc)^-1,opts)

figure(9)
opts.Xlim = [1 1e4];
bodemag(K_til*S_til*G_til,opts)
hold on
bodemag((W2)^-1,opts)

figure(10)
opts.Xlim = [1 1e4];
bodemag(K_til*S_til,opts)
hold on
bodemag((W2*Vsc^(-1)*Wsc^(-1))^-1,opts)



%% K plots
close all
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
opts.Ylim = [-100 20];

figure(7)
bodemag(M_til(1:2,1:2),opts)
hold on
bodemag(W1^-1,opts)
grid on
title('Output Sensitivity')


figure(8)
bodemag(M_til(1:2,3:4),opts)
hold on
bodemag(W1^-1,opts)
grid on
title('Process Sensitivity')


figure(9)
bodemag(M_til(3:4,1:2),opts)
hold on
bodemag(W2^-1,opts)
grid on
title('Control Sensitivity')


figure(10)
bodemag(M_til(3:4,3:4),opts)
hold on
bodemag(W2^-1,opts)
grid on
title('Complementairy Sensitivity')


%% Interaction

freq_V = logspace(0,3,10e3)
G_frf = squeeze(freqresp(G, freq_V*2*pi));


RGA = zeros(2,2,length(G_frf));
for i = 1:length(G_frf)
    RGA(:,:,i) = abs(G_frf(:,:,i).*inv(G_frf(:,:,i)).');
end

figure(1)
subplot(2,2,1)
semilogx(freq_V,squeeze(RGA(1,1,:)))
grid on
ylim([0 7])
ylabel('RGA magnitude')
subplot(2,2,2)
semilogx(freq_V,squeeze(RGA(1,2,:)))
grid on
ylim([0 7])
subplot(2,2,3)
semilogx(freq_V,squeeze(RGA(2,1,:)))
grid on
xlabel('Frequency (Hz)')
ylabel('RGA magnitude')
ylim([0 7])
subplot(2,2,4)
semilogx(freq_V,squeeze(RGA(2,2,:)))
xlabel('Frequency (Hz)')
grid on
ylim([0 7])
sgtitle('RGA of plant G')









