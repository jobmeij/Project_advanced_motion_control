clear all
close all
clc

% Parameters
m = 4.8e-6;
d = 1e-4;
c = 0.22;
ds = 1e-7;
cs = 1e-7;


G.a = [-(d+ds)/m -(c+cs)/m d/m c/m;
    1 0 0 0;
    d/m c/m -d/m -c/m;
    0 0 1 0];
G.b = [1/m 0 0 0]';
G.c = [ 0 0 0 1];
G.d = 0;
sys = ss(G.a,G.b,G.c,G.d);
Gtf = tf(sys);
systemcheck(sys)
eig(G.a)
if sum(eig(G.a)>0) == 0
    disp("system is stable")
end

figure(1)

opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
bode(sys,opts)

%% C

F_1 =  eye(1);
F_2 = -tf(sys);
F_3 = eye(1);
F_4 = -tf(sys);
P_1 = [F_1 F_2;F_3 F_4];

P_1ss = ss(G.a,[zeros(size(G.b,1),1) G.b],[-G.c;-G.c],[1 0;1 0]);

%% D

opts = hinfsynOptions('Display','on');
[K1,CL,gamma] =  hinfsyn((P_1ss),1,1,opts);
closed_loop = lft((P_1ss),K1);

figure(1)
bode(closed_loop)
hold on

figure(2)
bdopts = bodeoptions;
bdopts.FreqScale = 'Linear';
bode(closed_loop,bdopts)


%% F weighting functions
s = tf('s');
beta = 0.3;
Fbw = 5; %Hz to Rad/s
W2 = (s^2+4*pi*beta*Fbw*s+(2*pi*Fbw)^2)/(s+2*pi*Fbw/20)^2;
W2_ss = ss(W2);

opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
figure(1)
bode(W2,opts)
figure(2)
bode(W2^-1,opts)




%% G 

P2 = augw(sys,W2_ss,[],[]);

opts = hinfsynOptions('Display','on');
[K2,CL2,~] =  hinfsyn((P2),1,1,opts);
closed_loop2 = lft((P2),K2);


opts = bodeoptions;
opts.MagUnits = 'abs';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
figure(2)
bode(closed_loop2,opts)


opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
figure(3)
closed_loop2 = lft((P_1ss),K2);
bode(closed_loop2,opts)
hold on
bode(W2^-1)
legend('CL(P_1,K_2)','1/W_2')
grid on


%% L

s = tf('s');
beta = 0.3;
Fbw = 20; %Hz
W3 = (s^2+4*pi*beta*Fbw*s+(2*pi*Fbw)^2)/(s+((2*pi*Fbw)/20))^2;
W3_ss = ss(W3);

opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
figure(1)
bode(W3,opts)

P3 = augw(sys,W3_ss,[],[]);

opts = hinfsynOptions('Display','on');
[K3,CL3,gamma] =  hinfsyn(P3,1,1,opts);

closed_loop3 = lft(P3,K3);

opts = bodeoptions;
opts.MagUnits = 'abs';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
figure(2)
bode(closed_loop3,opts)

opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
figure(3)
closed_loop4 = lft((P_1ss),K3);
bode(closed_loop4,opts)
hold on
bode(W3^-1)
legend('CL(P_1,K_3)','1/W_3')
grid on

%% n Perdurbed plant
% Parameters
c = 0.13;

Gp.a = [-(d+ds)/m -(c+cs)/m d/m c/m;
    1 0 0 0;
    d/m c/m -d/m -c/m;
    0 0 1 0];
Gp.b = [1/m 0 0 0]';
Gp.c = [ 0 0 0 1];
Gp.d = 0;
sysp = ss(Gp.a,Gp.b,Gp.c,Gp.d);
Gptf = tf(sysp);

Fp_1 =  eye(1);
Fp_2 = -tf(sysp);
Fp_3 = eye(1);
Fp_4 = -tf(sysp);
Pp = [Fp_1 Fp_2;Fp_3 Fp_4];


clG3 = lft((P_1),K3);
clG2 = lft((P_1),K2);

clGp3 = lft(Pp,K3);
clGp2 = lft(Pp,K2);

figure(1)
T = 0:0.001:1; %Time vector
step(clG2,T);
hold on
step(clG3,T);
step(clGp2,T);
legend('Nominal plant K2','Nominal plant K3','Perturbed plant K2')

figure(2)
step(clGp3,T);
legend('Perturbed plant K3')


%% o,p,q mixed syn
clear all
close all
clc

% Nominal plant
% Parameters
m = 4.8e-6;
d = 1e-4;
c = 0.22;
ds = 1e-7;
cs = 1e-7;

G.a = [-(d+ds)/m -(c+cs)/m d/m c/m;
        1 0 0 0;
        d/m c/m -d/m -c/m;
        0 0 1 0];
G.b = [1/m 0 0 0]';
G.c = [ 0 0 0 1];
G.d = 0;
sys = ss(G.a,G.b,G.c,G.d);
Gtf = tf(sys);

F_1 =  eye(1);
F_2 = -tf(sys);
F_3 = eye(1);
F_4 = -tf(sys);
P_1 = [F_1 F_2;F_3 F_4];
P_1ss = ss(G.a,[zeros(size(G.b,1),1) G.b],[-G.c;-G.c],[1 0;1 0]);

% Perdurbed plant
% Parameters
c = 0.13;

Gp.a = [-(d+ds)/m -(c+cs)/m d/m c/m;
    1 0 0 0;
    d/m c/m -d/m -c/m;
    0 0 1 0];
Gp.b = [1/m 0 0 0]';
Gp.c = [ 0 0 0 1];
Gp.d = 0;
sysp = ss(Gp.a,Gp.b,Gp.c,Gp.d);
Gptf = tf(sysp);

Fp_1 =  eye(1);
Fp_2 = -tf(sysp);
Fp_3 = eye(1);
Fp_4 = -tf(sysp);
Pp = [Fp_1 Fp_2;Fp_3 Fp_4];

s = tf('s');
%Parameters:
Fbw = 9; % 5Hz 
Beta1 = 0.5;
Beta2 = 0.5;
W1 = 2*pi*0.0015;
W2 = 2*pi*1000;

%Performance weight:
Wp = (s+2*pi*Fbw)^2/(2*(s+2*pi*Fbw/10)^2);

%Relative error model:
Wi = 2000*(s^2+2*Beta1*W1*s+W1^2)/(s^2+2*Beta2*W2*s+W2^2);

%Plot the bound on model:
figure(1)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
bodemag(Wi,opts)
hold on 
bodemag((Gtf-Gptf)/Gtf,opts)

%H_inf norm of closed loop weighted mixed sensitivity
[K1,~,GAM,INFO] = mixsyn(sys,Wp,[],Wi); 

%Plot the controller
figure(2)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
bodemag(K1,opts)

%Testing for RP
figure(3)
S =(1+sys*K1)^-1;
T = 1-S;
KS = K1*S;
SVs = sigma(Wp*S);
SVt = sigma(Wi*T);
SVks = sigma(KS);
max([SVs SVt SVks]);
GAM
N = [Wp*S;Wi*T;KS];
norm(N,2)

sigma(Wp*S,'b',Wi*T,'r',KS,'g',{1e-3,1e3});
legend('Wp*S','Wi*T','KS','Location','best')
grid

%Closed loop transfer functions
CLnp = lft((P_1),K1);
CLpp = lft((Pp),K1);
figure(6)
opts = bodeoptions;
opts.MagUnits = 'dB';
opts.FreqUnits = 'Hz';
opts.Xlim = [1 1e3];
bodemag(CLnp,opts);
hold on
bodemag(CLpp,opts);
bodemag(1/Wp,opts);
legend('Closed loop nominal plant','Closed loop perturbed plant','1/Wp')


%Step responses nominal plant and perturbed plant
figure(4)
T = 0:0.001:1; %Time vector
step(CLnp,T);
hold on
step(CLpp,T);
legend('Closed loop nominal plant','Closed loop perturbed plant')

systemcheck(sys)
eig(G.a)
if sum(eig(G.a)>0) == 0
    disp("plant is stable")
end
K = tf(K1);
G = tf (sys);
Q = K*(1 + K*G)^-1

%%