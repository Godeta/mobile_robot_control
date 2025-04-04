syms t

xr=2*sin(((2*pi)/30)*t);
yr=2*sin(((4*pi)/30)*t);

dxr=diff(xr,t)
dyr=diff(yr,t)

dxr2=diff(dxr,t)
dyr2=diff(dyr,t)