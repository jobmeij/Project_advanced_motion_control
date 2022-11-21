%% Advanced motion control - exercise 8
clear all; close all; clc;

% Load data
load('xyztable.mat')

% Variables
a = 0.1;
b = a;

%% 8c
% - Determine a solution that minimizes the actuator forces in terms of the
%       2-norm (SVD?)
% - The z sensors are all subject to noise, which is independent and
%       normally distributed.
% - Precisely specify the solution that is obtained, using pinv in Matlab
%       is insufficient.

% Estimate model
Response = G.ResponseData;
Freq = G.Frequency;
Ts = G.Ts;
Gdata = idfrd(Response,Freq,Ts);
PolesX = 2;
PolesY = 2;
PolesZ = 4;
np =   [PolesX PolesX 0 0 0 0 0 0;
    0 0 PolesY PolesY 0 0 0 0;
    0 0 0 0 PolesZ PolesZ PolesZ PolesZ;
    0 0 0 0 PolesZ PolesZ PolesZ PolesZ;
    0 0 0 0 PolesZ PolesZ PolesZ PolesZ;
    0 0 0 0 PolesZ PolesZ PolesZ PolesZ];
Gest = tfest(Gdata,np);

figure()
bodemag(Gest)
grid on
title('Magnitude of estimated transfer matrix G_e_s_t(s)')

% Computing Ty
Ty =   [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 0.25 0.25 0.25 0.25];

% Computing Tu at lowest possible frequency (steady state)
Tu = pinv(Ty*G.ResponseData(:,:,1));

Gdec = Ty*Gest*Tu;

figure()
bodemag(Gdec)
grid on
title('Steady-state decoupled system')

% Computing Ginv
Ginv = frd(1/G.ResponseData,G.Frequency);

figure()
bodemag(Ginv)
grid on
title('Magnitude of inverted G(s)')

%%
g = tf([1],[1 0 0])
% Gtilde =   [0.01 0.01 0 0 0 0 0 0;
%             0 0 0.01 0.01 0 0 0 0;
%             0 0 0 0 0.33 0.33 0.33 0.33;
%             0 0 0 0 0.33 0.33 0.33 0.33;
%             0 0 0 0 0.33 0.33 0.33 0.33;
%             0 0 0 0 0.33 0.33 0.33 0.33];
Gtilde = G.ResponseData(:,:,1)
Gs = g*Gtilde

% A = Response(:,:,1);
A = Gtilde;

Ty = A'*((A*A')^-1)     % right inverse
Tu = ((A'*A)^-1)*A'     % left inverse



%% 8d: design of a decenteralized controller K1 for the decoupled system in x,y and z directions
% Requirements:
% - Closed-loop stability (check margins, characteristic loci, eigenvalues)
% - Cross-over frequency of 80 Hz (open loop cross over of 80 Hz)
% - Good low-frequency disturbance attenuation (= high open-loop gain at
%       lower frequencies)

% Insert decoupled system:
% Computing Ty
Ty =   [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 0.25 0.25 0.25 0.25];
% Computing Tu at lowest possible frequency (steady state)
Tu = pinv(Ty*G.ResponseData(:,:,1));
% Rewrite to decoupled plant:
Gdec = Ty*Gest*Tu;

% Set controller
% Lead filter
fc = 80;            % Desired cross-over frequency
f1 = 20;            % Start frequency of lead
f2 = (fc^2)/f1      % Computed stop frequency of lead (from start and cross-over freq)
leadFilt = tf([(1/(2*pi*f1)) 1],[(1/(2*pi*f2)) 1])
figure()
bode(leadFilt)
grid on

K1 = -158000*leadFilt*eye(3);

% Compute open- and closed loop transfers
L = Gdec*K1;
T = L*inv(eye(length(L))+L)
S = inv(eye(length(L))+L)

% Plot open loop
% figure()
% bode(L(1,1),10:10000);
% hold on
% bode(L(2,2),10:10000);
% bode(L(3,3),10:10000);
% hold off
% grid on

% figure()
% step(T)
% grid on

% Plot open loop transfers
figure()
bodemag(L(1,1),1:10000)
hold on
bodemag(L(2,2),1:10000)
bodemag(L(3,3),1:10000)
hold off
grid on
title('Open loop transfer of G_d_e_c(s)K_1(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

% Plot closed loop transfers
figure()
bodemag(T(1,1),1:10000)
hold on
bodemag(T(2,2),1:10000)
bodemag(T(3,3),1:10000)
hold off
grid on
title('Closed loop transfer of G_d_e_c(s)K_1(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

% Plot sensitivity
figure()
bodemag(S(1,1),1:10000)
hold on
bodemag(S(2,2),1:10000)
bodemag(S(3,3),1:10000)
hold off
grid on
title('Sensitivity of G_d_e_c(s)K_1(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

% Plot nyquist
figure()
hold on
nyquist(L(1,1))
nyquist(L(2,2))
nyquist(L(3,3))
hold off
legend('Loop x','Loop y','Loop z','Location','Best')
xlim([-1.1 0.1])
ylim([-2 2])

%% 8e controller with fco = 220 Hz

% Set controller
% Lead filter
fc = 220;            % Desired cross-over frequency
f1 = 60;            % Start frequency of lead
f2 = (fc^2)/f1      % Computed stop frequency of lead (from start and cross-over freq)
leadFilt = tf([(1/(2*pi*f1)) 1],[(1/(2*pi*f2)) 1])
figure()
bode(leadFilt)
grid on

K2 = -158000*8.3*leadFilt*eye(3);

% Compute open- and closed loop transfers
L2 = Gdec*K2;
T2 = L2*inv(eye(length(L2))+L2)
S2 = inv(eye(length(L2))+L2)

% Plot open loop
figure()
bode(L2(1,1),10:100000);
hold on
bode(L2(2,2),10:100000);
bode(L2(3,3),10:100000);
hold off
grid on
title('Open loop transfer of G_d_e_c(s)K_2(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

% figure()
% step(T2)
% grid on

% Plot open loop transfers
figure()
bodemag(L2(1,1),1:100000)
hold on
bodemag(L2(2,2),1:100000)
bodemag(L2(3,3),1:100000)
hold off
grid on
title('Open loop transfer of G_d_e_c(s)K_2(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

% Plot closed loop transfers
figure()
bodemag(T2(1,1),1:100000)
hold on
bodemag(T2(2,2),1:100000)
bodemag(T2(3,3),1:100000)
hold off
grid on
title('Closed loop transfer of G_d_e_c(s)K_2(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

% Plot sensitivity
figure()
bodemag(S2(1,1),1:100000)
hold on
bodemag(S2(2,2),1:100000)
bodemag(S2(3,3),1:100000)
hold off
grid on
title('Sensitivity of G_d_e_c(s)K_2(s)')
legend('Loop x','Loop y','Loop z','Location','Best')

figure()
hold on
nyquist(L2(1,1))
nyquist(L2(2,2))
nyquist(L2(3,3))
hold off
% grid on
title('Nyquist diagram of open loop G_d_e_c(s)K_2(s)')
legend('Loop x','Loop y','Loop z')
xlim([-1.1 0.1])
ylim([-2 2])


%% Create characteristic loci for controller 1 and 2 (work in progress)
% freq = logspace(1,3,10e3);
freq = G.Frequency / (2*pi);

poles = pole(T2);
poles_ol = pole(L2);

figure()
plot(real(poles),imag(poles),'X');
title('Closed loop poles')
grid on

figure()
plot(real(poles_ol),imag(poles_ol),'X');
title('Open loop poles')
grid on

i = 1;
for f = freq'
    f = f
    L_freq = (Ty * G.ResponseData(:,:,i) * Tu) * evalfr(K2,f);
    Loci(i) = prod(1+eig(L_freq));
    %     Loci2(i,:) = eig(L_freq);
    i = i+1;
end

figure()
hold on
plot(real(Loci),-imag(Loci))
hold off
grid on
title('Characteristic loci of L2')
xlabel('Real')
ylabel('Imag')


% for w = freq
%     L = [((1i*w)+10)/((1i*w)^2+(1i*w)+10) 10/((1i*w)^4+(1i*w)^3+10*(1i*w)^2+(1i*w)+1);
%         alpha/((1i*w)^2+0.1*(1i*w)+10) ((1i*w)+10)/(0.1*(1i*w)^2+(1i*w)+10)];
%     Loci(i) = prod(1+eig(L));
%     Loci2(i,:) = eig(L);
%     i = i+1;
%
% end
% figure()
% lociplot(alpha+1)= plot(Loci,'Color',colors(alpha+1,:));
% hold on
% plot(real(Loci),-imag(Loci),'Color',colors(alpha+1,:))
% grid on

%% Modal decoupling
close all

% Create matrix to scale output of G to desired decoupled output
Ty2 =  [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 Phi(:,1)';
    0 0 Phi(:,2)';
    0 0 Phi(:,3)']';

% Invert Ty2 because it is determined from its output to its input
Ty2 = pinv(Ty2)

% Compute Tu2 based on the inverse of G (steady state) and Ty
Tu2 = pinv(Ty2*G.ResponseData(:,:,1));

Gdec2 = zeros(5,5,length(G.Frequency));
% Rewrite to decoupled plant:
for n = 1:length(G.Frequency)
    Gdec2(:,:,n) = round(Ty2*G.ResponseData(:,:,n)*Tu2,10);  % Implemented rounding to remove low values from bode
end

% Create frd data from computed decoupled response
Gdec2_frd = frd(Gdec2,G.Frequency)

bOptions = bodeoptions;
% bOptions.YLim([-60 20])

% Plot bode
figure()
bodemag(Gdec2_frd,bOptions)
grid on

%% Question 8g: designing a decentralized controller K_modal for the system of 8f realizing a bandwidth of 220 Hz for the x, y and z(rb) loops
close all; clc;
K_modal =  [4.84*10e5 0 0 0 0;
    0 4.84*10e5 0 0 0;
    0 0 4.84*10e5 0 0;
    0 0 0 -4 0;
    0 0 0 0 -4];

% Resonance peak frequency flexible z axis loop 1: 150 Hz
fc = 300;            % Desired cross-over frequency
f1 = 100;            % Start frequency of lead
f2 = (fc^2)/f1      % Computed stop frequency of lead (from start and cross-over freq)
leadFiltFlex1 = tf([(1/(2*pi*f1)) 1],[(1/(2*pi*f2)) 1])

% Resonance peak frequency flexible z axis loop 2: 820 Hz
fc = 1200;            % Desired cross-over frequency
f1 = 600;            % Start frequency of lead
f2 = (fc^2)/f1      % Computed stop frequency of lead (from start and cross-over freq)
leadFiltFlex2 = tf([(1/(2*pi*f1)) 1],[(1/(2*pi*f2)) 1])

% Controller
a = eye(5);
a(4:5,4:5) = 0;
leadFiltModal = (leadFilt * a);
leadFiltModal(4,4) = leadFiltFlex1;
leadFiltModal(5,5) = leadFiltFlex2;

K_modal = -0.27 * leadFiltModal * K_modal;

% Setting bode to Hz
bOptions = bodeoptions;
bOptions.FreqUnits = 'Hz';

% Creating open loop, closed loop and sensitivity functions
L_modal = Gdec2_frd * K_modal;
T_modal = L_modal*inv(eye(length(L_modal))+L_modal);
S_modal = inv(eye(length(L_modal))+L_modal);

% Plot open loop transfers
figure()
bodemag(L_modal(1,1),2*pi*(1:10000),bOptions)
hold on
bodemag(L_modal(2,2),2*pi*(1:10000),bOptions)
bodemag(L_modal(3,3),2*pi*(1:10000),bOptions)
bodemag(L_modal(4,4),2*pi*(1:10000),bOptions)
bodemag(L_modal(5,5),2*pi*(1:10000),bOptions)
hold off
grid on
title('Open loop transfer of G_{modal}(s)K_{modal}(s)')
legend('Loop x','Loop y','Loop z(rd)','Loop z(flex1)','Loop z(flex2)','Location','Best')

% Plot closed loop transfers
figure()
bodemag(T_modal(1,1),2*pi*(1:10000),bOptions)
hold on
bodemag(T_modal(2,2),2*pi*(1:10000),bOptions)
bodemag(T_modal(3,3),2*pi*(1:10000),bOptions)
bodemag(T_modal(4,4),2*pi*(1:10000),bOptions)
bodemag(T_modal(5,5),2*pi*(1:10000),bOptions)
hold off
grid on
title('Closed loop transfer of G_{modal}(s)K_{modal}(s)')
legend('Loop x','Loop y','Loop z(rd)','Loop z(flex1)','Loop z(flex2)','Location','Best')

% Plot sensitivity
figure()
bodemag(S_modal(1,1),2*pi*(1:10000),bOptions)
hold on
bodemag(S_modal(2,2),2*pi*(1:10000),bOptions)
bodemag(S_modal(3,3),2*pi*(1:10000),bOptions)
bodemag(S_modal(4,4),2*pi*(1:10000),bOptions)
bodemag(S_modal(5,5),2*pi*(1:10000),bOptions)
hold off
grid on
title('Sensitivity of G_{modal}(s)K_{modal}(s)')
legend('Loop x','Loop y','Loop z(rd)','Loop z(flex1)','Loop z(flex2)','Location','Best')

% Plot nyquist
figure()
hold on
nyquist(L_modal(1,1))
nyquist(L_modal(2,2))
nyquist(L_modal(3,3))
nyquist(L_modal(4,4))
nyquist(L_modal(5,5))
hold off
legend('Loop x','Loop y','Loop z(rd)','Loop z(flex1)','Loop z(flex2)','Location','Best')
xlim([-5 1])
ylim([-5 5])

% Estimate transfer functions
np_modal = [3 0 0 0 0;
    0 3 0 0 0;
    0 0 3 0 0;
    0 0 0 5 0;
    0 0 0 0 5];
Gest_modal = tfest(Gdec2_frd,np_modal)
Gest_modal_L = tfest(L_modal,np_modal)
Gest_modal_T = tfest(T_modal,np_modal)

% Plant bode
figure()
bode(Gest_modal,bOptions)
grid on
title('Bode of estimated G_{modal}')

% Open loop bode
figure()
bode(Gest_modal_L,1:10000,bOptions)
grid on
title('Bode of estimated L_{modal}')

% Closed loop bode
figure()
bode(Gest_modal_T,1:1000,bOptions)
grid on
title('Bode of estimated T_{modal}')

% Step response
figure()
step(Gest_modal_T)
grid on
title('Step closed loop with K_{modal}')

%% Question 8h: implementing the controllers on plant G instead of TyGTu/Ty2GTu2
% Compute the controllers K1_org, K2_org, Kmodal_org
% - are the resulting controllers diagonal?
% - what is their McMillan degree? (compute minreal size)

% Using K1 to control G(s) directly by changing inputs
K1org =    [10^(32/20)*K1(1,1) 10^(32/20)*K1(1,1) 0 0 0 0 0 0;
    0 0 10^(32/20)*K1(2,2) 10^(32/20)*K1(2,2) 0 0 0 0;
    0 0 0 0 K1(3,3) K1(3,3) K1(3,3) K1(3,3);
    0 0 0 0 K1(3,3) K1(3,3) K1(3,3) K1(3,3);
    0 0 0 0 K1(3,3) K1(3,3) K1(3,3) K1(3,3);
    0 0 0 0 K1(3,3) K1(3,3) K1(3,3) K1(3,3)];

L1org = -1*K1org.*Gest;

% Plot open loop K2(s)G(s)
figure()
bode(L1org,bOptions);
grid on
title('Open-loop response of K1org applied to G(s)')

% Using K2 to control G(s) directly by changing inputs
K2org =    [10^(32/20)*K2(1,1) 10^(32/20)*K2(1,1) 0 0 0 0 0 0;
    0 0 10^(32/20)*K2(2,2) 10^(32/20)*K2(2,2) 0 0 0 0;
    0 0 0 0 K2(3,3) K2(3,3) K2(3,3) K2(3,3);
    0 0 0 0 K2(3,3) K2(3,3) K2(3,3) K2(3,3);
    0 0 0 0 K2(3,3) K2(3,3) K2(3,3) K2(3,3);
    0 0 0 0 K2(3,3) K2(3,3) K2(3,3) K2(3,3)];

L2org = -1*K2org.*Gest;

% Plot open loop K2(s)G(s)
figure()
bode(L2org,bOptions);
grid on
title('Open-loop response of K2org applied to G(s)')

X = 10^(32/20); % Gain
Y = K_modal(3,3) * K_modal(4,4);
Z = K_modal(3,3) * K_modal(5,5);
B = K_modal(3,3);
% Kmodalorg =    [X*K_modal(1,1) X*K_modal(1,1) 0 0 0 0 0 0;
%     0 0 X*K_modal(2,2) X*K_modal(2,2) 0 0 0 0;
%     0 0 0 0 Y Y 0 0;
%     0 0 0 0 Y Y 0 0;
%     0 0 0 0 0 0 Z Z;
%     0 0 0 0 0 0 Z Z];

Kmodalorg =    [X*K_modal(1,1) X*K_modal(1,1) 0 0 0 0 0 0;
    0 0 X*K_modal(2,2) X*K_modal(2,2) 0 0 0 0;
    0 0 0 0 B B B B;
    0 0 0 0 B B B B;
    0 0 0 0 B B B B;
    0 0 0 0 B B B B];

Lmodalorg = -1*Kmodalorg.*Gest;

figure()
bode(Lmodalorg,bOptions);
grid on
title('Open-loop response of Kmodal org applied to G(s)')


%% Question 8i: sinusoidal output disturbance is present at sensor z3
% - Determine the transfer function that quantifies the disturbance
% attenuation properties in the low frequency range and at approximately
% 150 Hz
% - Take into account the design choices that are made when designing the
% controllers






