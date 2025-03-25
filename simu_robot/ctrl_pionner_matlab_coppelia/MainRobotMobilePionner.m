close all;clear all;clc;

global  d r1 dt
% robot parameters
d = 0.1950; % wheel radius
r1 = d/2; % wheel radius

his.dt=[];
dt=0.15;
x=0;y=0;theta=0;
X=[x;y;theta];

%positions à atteindre
%xr=[-1,1,1,-1]; 
%yr=[-1,-1,1,1];

%interpolation
% xr=[0,-1,1,0.9,-1.1];
% yr=[0,-1,-0.9,1,1.1];
% temps1 = [0,1,2,3,4];
% new_t=0:0.1:max(temps1);
% xr=interp1(temps1,xr,new_t);
% yr=interp1(temps1,yr,new_t);

tperiod = 2*[0 2.5 5 10 15 20 22.5 25];
points = [0 0; 1 0; 1 1; -1 1;-1 -1; 1 -1; 1 0; 0 0];
x_points = points(:,1);
y_points = points(:,2);

t = 0:dt:tperiod(end);
xr=interp1(tperiod,x_points,t);
yr=interp1(tperiod,y_points,t);

%trajectoire de cercle :
%xr = cos(t)
%yr = sin(t)

%tperiod = [10 15 20 25];
%x_des=interp1(tperiod,xr,t)

%interpolation
% xtest = 0:-pi/4:-1.5*pi; 
% vtest = sin(xtest);
% xqtest = 0:-pi/16:-1.5*pi;
% vq1 = interp1(xtest,vtest,xqtest);
% xr=vq1;
% yr=vq1;

Vref=0;        % max 0.5 m/s
Omegaref=0.0;    % max 3 rps
%t = 0:dt:5; %temps
his.X=[];his.Xr=[];his.x=[];his.y=[];his.theta=[];his.Vref=[];his.Omegaref=[];his.vr=[];his.wr=[];his.thetar=[];

%%  Graphics
f3=figure;
f3.Position = [50 75 1450 700];
%f3.Position = [90 -20 4000 100];
subplot(5,3,[1,14]), %taille schema
%ax3=axes('parent',f3);

hold on;grid on;
rectangle('Position',[-2.5,-2.5,5,5],'FaceColor','white','EdgeColor','#3f3f3f','LineWidth',5),


plot(0, 0,'r*','LineWidth',15);
plot(xr, yr,'b*','LineWidth',15);

hold on;grid on;
PLOT.RefTrajectory=line('XData',[],'YData',[],'LineStyle','-.','color','r','LineWidth',1);
PLOT.Robot=line('XData',[],'YData',[],'LineStyle','--','color','b','LineWidth',1);
xlabel('x (m)')
ylabel('y (m)')
%legend('Reference Trajectory','Acual')
grid on


%  robot dimensions
A.R_w =0.1950/2; % robot width/2
A.R_l=0.1950/2;   % robot length/2
A.a1 = [-A.R_l -A.R_w]';
A.b1 = [A.R_l -A.R_w]';
A.b2 = [A.R_l A.R_w]';
A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c];
pl=[];

A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotated car
A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; % add offset of car's center

A.P_robot=patch(A.P(1,:),A.P(2,:),'k');
A.P_robot.XData=A.Prot_trasl(1,:)';
A.P_robot.YData=A.Prot_trasl(2,:)';
axis([-2.5 2.5 -2.5 2.5]); %visuel graphique matlab axe

%% Main loop
vrep=remApi('remoteApi'); vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,1);
connected = false;
if (clientID>-1)
connected = true;
disp('Connected to remote API server');
%% ---------------------------------start simulation-----------------------
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);   
%%
[~,Robot] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
[~,Pioneer_p3dx_rightMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
[~,Pioneer_p3dx_leftMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);

[~,Position] = vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_streaming);
[~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_streaming);
[~,Position] = vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer);
[~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_buffer);

end


 if (connected) 
     
%affichage infos
legend("origine", "destination", "consigne", "trajet", "robot");

    
    %boucle déplacement
    for i=1:length(t)
    
    [~,Position] = vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer);
    [~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_buffer);
    %% ----------------------------Temps---------------------------------------
    %% ----------------------Variables d'etat----------------------------------
    x=Position(1);
    y=Position(2);
    theta=Orientation(3);
    X=[x,y,theta];
    %% ----------------------Control----------------------------------
    B=sqrt((xr(i)-x)^2 + (yr(i)-y)^2);
    angle=atan2(yr(i)-y,xr(i)-x);
    a=angle-theta;
    a = AngleWrap(a);
    y=angle+Omegaref;
    
    %alpha1= atan2(xr-x,yr-y)
    %gamma= (w-ka*alpha1)/ky
    
    %controleur
    kb=0.5;
    ky=0;
    ka=2.2;
    
    [v,w]=CalculateControlOutput(kb, B, ka, a, ky, y)
    V_r = (2*v+d*w)/(2*r1);
    V_l = (2*v-d*w)/(2*r1);
    %[v_r, v_l] = CalculVitesseRoues(v,w);

    %% déplacement
    
    vrep.simxSetJointTargetVelocity(clientID,Pioneer_p3dx_rightMotor,V_r,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,Pioneer_p3dx_leftMotor,V_l,vrep.simx_opmode_blocking);
    
    
        his.X=[his.X X];   
        XX(i)=X(1);
        YY(i)=X(2);
        
       
        
        
        %set(PLOT.RefTrajectory,'XData',x_des(1:i),'YData',y_des(1:i));
        set(PLOT.Robot,'XData',XX,'YData',YY); 
        A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotated car
        A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; % add offset of car's center
        A.P_robot.XData=A.Prot_trasl(1,:)';
        A.P_robot.YData=A.Prot_trasl(2,:)';
        drawnow 
       pause(dt)
    end  

[~]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
pause(dt);
%% ---------------------------------stop simulation------------------------   
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
  end
vrep.delete(); % call the destructor!

%% ----- Tracer les résultats
