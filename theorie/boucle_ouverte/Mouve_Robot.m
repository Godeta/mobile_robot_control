function [ Xnew ] = Mouve_Robot(X,u,dt)

x=X(1);y=X(2);theta=X(3);
%%
V=u(1);
Omega=u(2);

x=x+V*dt*(cos(theta));
y=y+V*dt*(sin(theta));
theta=theta+Omega*dt;

Xnew=[x;y;theta];
end

