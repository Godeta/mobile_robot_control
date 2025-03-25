x = 0:-pi/4:-1.5*pi; 
v = sin(x);
xq = 0:-pi/16:-1.5*pi;

vq1 = interp1(x,v,xq);
figure
plot(x,v,'o',xq,vq1,':.');
xlim([-pi 2]);
title('(Default) Linear Interpolation');