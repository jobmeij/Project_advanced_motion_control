%% Control of an Active Vibration Isolation System (AVIS)
clear all
close all
clc

load('avis.mat');
s = tf('s');
K = 1/(s+(100*pi));

S_mod = 1/(1+G_mod*K);
T_mod = G_mod*K/(1+G_mod*K);

figure(1)
bode(S_mod)
title('sensitivity bode')


figure(2)
bode(T_mod)
title('PS bode')

figure(3)
bode(K)
title('openloop bode')
