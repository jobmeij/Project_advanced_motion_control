clear all
close all
clc

s = tf('s');
G = [1/(s+1) 1/(s-2);
    0 1/(s+3)];
K = 4*eye(2);

figure(1)
nyquist(4*G(1,1))
title('First diagonal element L(s)')

figure(2)
nyquist(4*G(2,2))
title('Second diagonal element L(s)')


%Loci
%Loci1 = (4*s+12)/((s+1)*(s+3));
Loci1 = 4/(s+1);
%Loci2 = (4*s+4)/((s+1)*(s+3));
Loci2 = 4/(s+3);
freq = logspace(-1,5,10e3);



figure(3)
Lociplt1= squeeze(freqresp(Loci1,freq));
plot(Lociplt1)
hold on
plot(real(Lociplt1),-imag(Lociplt1))
title('Characteristic Loci \lambda_1(j\omega)')
xlabel('Real \lambda_1 L(s)')
ylabel('Imag \lambda_1 L(s)')
legend('Positive frequency','Negative frequency')
grid on

figure(4)
Lociplt2= squeeze(freqresp(Loci2,freq));
plot(Lociplt2)
hold on
plot(real(Lociplt2),-imag(Lociplt2))
title('Characteristic Loci \lambda_2(j\omega)')
xlabel('Real \lambda_2 L(s)')
ylabel('Imag \lambda_2 L(s)')
legend('Positive frequency','Negative frequency')
grid on

figure(5)
loci = zeros(1,length(Lociplt2));
for i = 1:length(Lociplt2)
    loci(i) = prod([1+Lociplt2(i) 1+Lociplt1(i)]);
end
plot(loci)
















%% B
clear all
close all
clc

s = tf('s');
L = [1/(s+1) 1;
    (s-1)/(s+1) 1];
eig(L) 

w = logspace(-2,10,1000);
s = 1i*w;
eig1 = (-s+(5*s.^2+4*s).^(1/2))./(2*(s+1));
eig2 = (-s-(5*s.^2+4*s).^(1/2))./(2*(s+1));

figure(1)
hold on
%plot(real(eig1),-imag(eig1)) %negative freq
plot(real(eig1),imag(eig1)) %Positive freq
cr = linspace (0,2*pi,100);
plot(cos(cr),sin(cr),'-.', 'Color','red')
% plot([-1 -1],[2 -2],'Color','r')
% plot([1 1],[2 -2],'Color','r')
xlabel('Real \lambda_1 L(s)')
ylabel('Imag \lambda_1 L(s)')
title('Eigenvalue \lambda_1 L(s)')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
grid on

figure(2)
hold on
%plot(real(eig2),-imag(eig2)) %negative freq
plot(real(eig2),imag(eig2)) %Positive freq
plot(cos(cr),sin(cr),'-.', 'Color','red')
% plot([-1 -1],[2 -2],'Color','r')
% plot([1 1],[2 -2],'Color','r')
xlabel('Real \lambda_2 L(s)')
ylabel('Imag \lambda_2 L(s)')
title('Eigenvalue \lambda_2 L(s)')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
grid on

% figure(3)
% nyquist(L)
% step(L)

%% C_a
close all
clear all
clc

s = tf('s');
L = (s+1)^2/s^3;
T = L*(1+L)^-1;

figure(1)
step(L)
title('step on open loop')

figure(2)
step(T)
title('closed loop')

figure(3)
nyquist(L)



%% C_b
clear all
close all
clc 

s = tf('s');
L = [ 1/(s+1) 1/s^3;
    0   1/(s+2)];

T = L*(1+L)^-1;

figure(1)
step(L)
title('step on open loop')

figure(2)
step(T)
title('closed loop')

w = logspace(-2,10,1000);
s = 1i*w;
eig1 = 1./(s+1);
eig2 = 1./(s+2);

figure(3)
hold on
% plot(real(eig1),-imag(eig1)) %negative freq
plot(real(eig1),imag(eig1)) %Positive freq
cr = linspace (0,2*pi,100);
plot(cos(cr),sin(cr),'-.', 'Color','red')
% plot([-1 -1],[2 -2],'Color','r')
% plot([1 1],[2 -2],'Color','r')
xlabel('Real \lambda_1 L(s)')
ylabel('Imag \lambda_1 L(s)')
title('Eigenvalue \lambda_1 L(s)')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
grid on

figure(5)
hold on
% plot(real(eig2),-imag(eig2)) %negative freq
plot(real(eig2),imag(eig2)) %Positive freq
cr = linspace (0,2*pi,100);
plot(cos(cr),sin(cr),'-.', 'Color','red')
% plot([-1 -1],[2 -2],'Color','r')
% plot([1 1],[2 -2],'Color','r')
xlabel('Real \lambda_2 L(s)')
ylabel('Imag \lambda_2 L(s)')
title('Eigenvalue \lambda_2 L(s)')
xlim([-1.5 1.5])
ylim([-1.5 1.5])
grid on

figure(4)
loci = zeros(1,length(eig2));
for i = 1:length(eig2)
    loci(i) = prod([1+eig2(i) 1+eig1(i)]);
end
plot(loci)
hold on
plot(real(loci),-imag(loci))