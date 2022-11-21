function [ys,yf,us,uf,ts,tf] = mrsim(us,ts,P,F,h);

% Function MRSIM simulates the behavior of the continuous time system P at
% a small sampling sample time h/F. A large F can thus be used to approximate the continuous time response of the system. The input to the system us is the ILC input at the
% long sample time h, with time vector ts. The output of the file contains:
% ys: output at the long sample time h
% yf: output at the small sample time h/F
% us: input at the long sample time h
% uf: input at the small sample time h/F (here zero-order-hold interpolation is used)
% ts: time vector at long sample time h
% tf: time vector at small sample time h/F

% Created 2013-04-18, T. Oomen

tf = [ts(1):h/F:ts(end)+h-h/F]';
uf = [];
for i = 1:length(ts)
    uf = [uf; ones(F,1)*us(i)];
end

% dt system
Pdf = c2d(P,h/F);

yf = lsim(Pdf,uf,tf);

ys = yf(1:F:end);