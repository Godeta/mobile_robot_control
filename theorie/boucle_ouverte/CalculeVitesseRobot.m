function [ V ] = CalculeVitesseRobot(t)

%x=2*sin(((2*pi)/30)*t);
%y=2*sin(((4*pi)/30)*t);

%voir derivees_calcul
dxr =(2*pi*cos((pi*t)/15))/15;
dyr =(4*pi*cos((2*pi*t)/15))/15;

dxr2 = -(2*pi^2*sin((pi*t)/15))/225;
dyr2 =-(8*pi^2*sin((2*pi*t)/15))/225;


vr = sqrt(dxr^2+dyr^2);
wr=(dyr2*dxr-dxr2*dyr) / (dxr^2+dyr^2);

V=[vr;wr];
end

