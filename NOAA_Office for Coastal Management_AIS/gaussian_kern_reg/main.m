clear;close all;clc;

x = 1:100; % training data x
y = sin(x/10)+(x/50).^2+0.2*randn(1,100);   % training data y
h=5; % kernel bandwidth

for i=1:100
    xs(i)=i;
    ys(i)=gaussian_kern_reg(xs(i),x,y,h);
end

figure;hold on; 
plot(x,y,'.');
plot(xs,ys,'r-');

