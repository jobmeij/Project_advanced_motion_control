clear all
close all
clc

load('xyztable.mat');
plot(1)
bodemag(G)
grid on

%%

Ty = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      0 0 1/4 1/4 1/4 1/4];

Tu1 = pinv(Ty*G.ResponseData(:,:,1));
Tu2 = [1/2 0 0; 0 1/2 0 ; 0 0 1/4];

Gdec = Ty*G*Tu1;
figure(2)
bodemag(Gdec*Tu2)
hold on
bodemag(Gdec)
s = tf('s');

K = 20; 
tc = 0.1;

LP = K/(tc*s+1);
K1 = [LP 0 0;
      0 s 0;
      0 0 s];
L = K1*Gdec;
bodemag(L)


freq = G.Frequency;

for i = 1:length(freq)
        L2 = freqresp(L,freq(i));
        Loci(i,:) = eig(L2);    
end
figure(3)
plot(Loci)
