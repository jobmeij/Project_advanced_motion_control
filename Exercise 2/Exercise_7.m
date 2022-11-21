clear all
close all
clc

alphaV = 0:1:10;
legendatxt = [];
colors = jet(length(alphaV));
lociplot = zeros(1,length(alphaV));
for alpha = alphaV
    legendatxt = [legendatxt "alhpha "+alpha];
    freq = logspace(-5,5,10e3);
    i = 1;
    s = tf('s');
    L2 = [((s)+10)/((s)^2+(s)+10) 10/((s)^4+(s)^3+10*(s)^2+(s)+1);
        alpha/((s)^2+0.1*(s)+10) ((s)+10)/(0.1*(s)^2+(s)+10)];
    T = L2*(L2+eye(size(L2)))^-1;
    
    poles = pole(T);
    poles_ol = pole(L2);
    
    figure(2)
    plot(real(poles),imag(poles),'X','Color',colors(alpha+1,:));
    
    hold on
    
    figure(3)
    plot(real(poles_ol),imag(poles_ol),'X','Color',colors(alpha+1,:));
    hold on
    
    for w = freq
        L = [((1i*w)+10)/((1i*w)^2+(1i*w)+10) 10/((1i*w)^4+(1i*w)^3+10*(1i*w)^2+(1i*w)+1);
            alpha/((1i*w)^2+0.1*(1i*w)+10) ((1i*w)+10)/(0.1*(1i*w)^2+(1i*w)+10)];
        Loci(i) = prod(1+eig(L));
        Loci2(i,:) = eig(L);
        i = i+1;
        
    end
    figure(1)
    lociplot(alpha+1)= plot(Loci,'Color',colors(alpha+1,:));
    hold on
    plot(real(Loci),-imag(Loci),'Color',colors(alpha+1,:))
    grid on
    
    if alpha == 2
        figure(4)
        plot(Loci2,'Color',colors(alpha+1,:));
        hold on
        plot(real(Loci2),-imag(Loci2),'Color',colors(alpha+1,:))
        grid on
    end
end

L3 = [((s)+10)/((s)^2+(s)+10) 0; 0 ((s)+10)/(0.1*(s)^2+(s)+10)];
figure(5)
nyquist(L3(1,1)+1)
hold on
nyquist(L3(2,2)+1)

figure(1)
title('characteristic loci')
origin = plot(0,0,'x','Color','red');
legend([lociplot origin],[legendatxt "origin"])
xlabel('real')
ylabel('imag')

%hold on
%cr = linspace (0,2*pi,100);
%plot(1+cos(cr),sin(cr),'-.', 'Color','red')
grid on

figure(2)
title('poles of the closed loop')
legend(legendatxt)
xlabel('real')
ylabel('imag')
xlim([-1 1])
ylim([-5 5])
grid on

figure(3)
title('Poles of the open loop')
legend(legendatxt)
xlabel('real')
ylabel('imag')
xlim([-1 1])
ylim([-5 5])
grid on

figure(4)
title('characteristic loci alpha 0')
origin = plot(0,0,'x','Color','red');
xlabel('real')
ylabel('imag')

%hold on
%cr = linspace (0,2*pi,100);
%plot(1+cos(cr),sin(cr),'-.', 'Color','red')
grid on

