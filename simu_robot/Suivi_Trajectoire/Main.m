clear all;clc;close all;


%Dans cette Simulation, pour une position de référence donnée (xr, yr):
%la loi de commande conçue est basé sur les cordonnées poliare 
%pour conduire le robot de sa configuration actuelle, à la position cible


%----- déclaration de variables intiale du robot
x=0;y=0;theta=0;% Orientation et position initial du robot 
X=[x,y,theta];% vecteur d'etat initial

v=0;    % vitesse lineaire initiale m/s a t=0 calculer a (t=0)
w=0;  % vitesse anguaire initial  rad/s a t=0  calculer a (t=0)

%-----------------------Trajectoire de Référence---------------------------
tperiod = [0 2.5 10 12.5 15 20 22.5 25 35 37.5 40 45 47.5 50 60 62.5]; % interval de temps entre chaque point

P0=[0 -30];
%--------------------------------------------------------------------------
P1=[0 -20];P2=[40 -20];P3=[40 -10];P4=[30 -10];P5=[30 10];P6=[40 10];P7=[40 20];
%--------------------------------------------------------------------------
P8=[-40 20];P9=[-40 10];P10=[-30 10];P11=[-30 -10];P12=[-40 -10];P13=[-40 -20];

P14=P1;P15=P0;

points =10^-1.5*[P0;P1; P2; P3; P4 ;P5 ;P6 ;P7; P8; P9 ;P10; P11; P12 ;P13 ;P14 ;P15];% points
x_ponit = points(:,1);% 
y_ponit = points(:,2);
dt=0.05; % pas de calcul dt (modifier pour avoir une solution plus en moins exact)

t = 0:dt:tperiod(end);
xr = interp1(tperiod,x_ponit,t);% x désiré
yr = interp1(tperiod,y_ponit,t);% y désiré
thetar=0;

% xr=0.4*sin((2*pi*t)/30);
% yr= 0.3*sin((4*pi*t)/30);
% dxr =(2*pi*cos((pi*t)/15))/75;
% dyr =(pi*cos((2*pi*t)/15))/25;
% thetar=atan2(dyr,dxr);

% Points du carré
points_x = [0.2, 0.2, 0, -0.2, -0.2, -0.2, 0, 0.2];
points_y = [0, 0.15, 0.15, 0.15, 0, -0.15, -0.15, 0];

% Temps pour chaque point
tperiod = [0, 5, 10, 15, 20, 25, 30, 35];

dt = 0.05; % pas de calcul
t = 0:dt:tperiod(end);

% Interpolation des points pour créer la trajectoire
xr = interp1(tperiod, points_x, t, 'linear');
yr = interp1(tperiod, points_y, t, 'linear');

% --------------------variables a tracer à la fin--------------------------
his.X=[];his.theta=[];his.x=[];his.y=[];
his.v=[];his.w=[];his.thetar=[]; 
% -----------------figure contirnt toutes les variables
f3=figure;
f3.Position = [50 75 1450 700];
subplot(5,3,[1,14]),
%------------------------------Espace de travail---------------------------
hold on;grid on;
rectangle('Position',3*[-0.5,-0.6,1,1.2],'FaceColor','white','EdgeColor','#3f3f3f','LineWidth',5);% Espace de travail

%------------------Animation
hold on;grid on;
PLOT.RefTrajectory=line('XData',[],'YData',[],'LineStyle','-.','color','k','LineWidth',2);
PLOT.Robot=line('XData',[],'YData',[],'LineStyle','--','color','b','LineWidth',2);
xlabel('x (m)');ylabel('y (m)');grid on
plot(xr,yr,'.','LineWidth',1);hold on;

%  dimensions du robot 
A.R_w =0.07; % robot width/2
A.R_l=0.07;  % robot length/2
A.a1 = [-A.R_l -A.R_w]';A.b1 = [A.R_l -A.R_w]';A.b2 = [A.R_l A.R_w]';A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c];pl=[];

A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotation robot
A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; %
A.P_robot=patch(A.P(1,:),A.P(2,:),'r');A.P_robot.XData=A.Prot_trasl(1,:)';A.P_robot.YData=A.Prot_trasl(2,:)';
 
%-----------------------------Main loop------------------------------------ 
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1) % If no connection is established, client id will be -1.
    disp('Connected to remote API server');
    vrep.simxSynchronous(clientID,true);
 
[retCod,rob1] = vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_oneshot_wait);
[retCod,rob1LM] = vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_oneshot_wait);
[retCod,rob1RM] = vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_oneshot_wait);

% --------------------- Start the simulation
[returnCode] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);



xp0 = xr(1);yp0 =yr(1);zp0 = 1.9150e-02;
% Orientation
ap0 = 0;bp0 = 0;cp0 =pi/2;
[retCod] = vrep.simxSetObjectPosition(clientID,rob1,-1,[xp0,yp0,zp0],vrep.simx_opmode_oneshot);
[retCod] = vrep.simxSetObjectOrientation(clientID,rob1,-1,[ap0,bp0,cp0],vrep.simx_opmode_oneshot);
%% ---------------------------------------------------------------------------------

for i=1:length(t)
            
%--------------------------------------------------------------------------
[retCod,Position] = vrep.simxGetObjectPosition(clientID,rob1,-1,vrep.simx_opmode_oneshot_wait);
[retCod,Orientation] = vrep.simxGetObjectOrientation(clientID,rob1,-1,vrep.simx_opmode_oneshot_wait);


x = Position(1,1);y = Position(1,2);theta = Orientation(1,3);
X=[x,y,theta];

% --------------La loi de commande basée sur les cordonnées polaires----------
% Calcul des erreurs en coordonnées polaires
p = sqrt((xr(i)-x)^2 + (yr(i)-y)^2);  % Distance euclidienne au point actuel de la trajectoire

% Calcul de l'angle désiré (direction vers le point cible)
alpha = atan2(yr(i)-y, xr(i)-x) - theta;
% Normalisation de l'angle entre -pi et pi
alpha = atan2(sin(alpha), cos(alpha));

% Angle absolu
beta = theta;
% Normalisation
beta = atan2(sin(beta), cos(beta));

% Gains du contrôleur
k_rho = 0.4;    % Gain sur la distance (contrôle la vitesse linéaire)
k_alpha = 1.9;    % Gain sur l'erreur d'angle (contrôle la vitesse angulaire)
k_beta = -0.04;   % Gain sur l'angle absolu

% Calcul des commandes
v = k_rho * p;  % Vitesse linéaire proportionnelle à la distance
w = k_alpha * alpha + k_beta * beta;  % Vitesse angulaire

% Limitation des vitesses pour éviter des comportements trop brusques
v_max = 0.25;  % Vitesse linéaire maximale
w_max = 3.0;   % Vitesse angulaire maximale
%v = max(min(v, v_max), 0);  % Limitation positive seulement (toujours avancer)
%w = max(min(w, w_max), -w_max);

% Ajustement de la vitesse linéaire en fonction de l'erreur d'angle
% Réduit la vitesse si l'angle est trop grand pour éviter les dérapages
%v = v * (1 - abs(alpha) / pi);

% --------------------------------------control law------------------------
    
%    
D = 0.053;R = D/2;L = 0.021;

leftVel=(v - L*w)/R;
rightVel=(v + L*w)/R;

[retCod] = vrep.simxSetJointTargetVelocity(clientID,rob1LM,leftVel,vrep.simx_opmode_oneshot);
[retCod] = vrep.simxSetJointTargetVelocity(clientID,rob1RM,rightVel,vrep.simx_opmode_oneshot);

% ---------------------------------Fin-------------------------------------
   % ----- Enregistrement de données  sous forme de vecteurs
    
    %his.X=[his.X X];
    his.theta=[his.theta theta]; % Rotation theta
    his.v=[his.v v];% la commande vr (vitesse lineaire) appliqué au robot
    his.w=[his.w w]; %la commande wr (vitesse angulaire) appliqué au robot
    his.x=[his.x x]; %la Position x
    his.y=[his.y y]; %la Position y

% tracer line du robot et de la trajectoire de reference xr et yr
   
    set(PLOT.RefTrajectory,'XData',xr,'YData',yr,'LineWidth',1);
    set(PLOT.Robot,'XData',x,'YData',y,'LineWidth',1)
    
    A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotated car
    A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; % add offset of car's center
    A.P_robot.XData=A.Prot_trasl(1,:)';
    A.P_robot.YData=A.Prot_trasl(2,:)';
    drawnow 
    
    vrep.simxSynchronousTrigger(clientID);

end

    [returnCode] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
    %----------- Before closing the connection to V-REP,
    %---------make sure that the last command sent out had time to arrive.
    [returnCode] = vrep.simxGetPingTime(clientID);
    %-------- Now close the connection to V-REP:
    vrep.simxFinish(-1); % just in case, close all opened connections
    disp("Connection Closed....... ");
end






%------------------ tracer les vairables 
subplot(3,3,3),grid on;
plot(t,his.v,'r:','LineWidth',1),grid on;hold on;
xlabel('Time [s]','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')
ylabel('v [m/s]','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')
set(legend( '$$v_r $$','FontSize',12,'Interpreter','Latex','FontWeight','bold','Color','b','Location','southeast'))
legend('boxoff')
title('Vitesse Lineaire (m/s)','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')


subplot(3,3,6),grid on;
plot(t,his.w,'r:','LineWidth',1),grid on;hold on;
xlabel('Time [s]','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')
ylabel('w [rad/s]','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')
set(legend( '$$w_r $$','FontSize',12,'Interpreter','Latex','FontWeight','bold','Color','b','Location','southeast'))
legend('boxoff')
title('Vitesse angulaire (rad/s)','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')



subplot(3,3,9),grid on;
plot(t,his.x,'b:',t,xr,'r:',t,his.y,'g:',t,yr,'c:','LineWidth',1),grid on;hold on;
xlabel('Time [s]','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')
ylabel('x [m]','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')

set(legend( '$$x $$','$$x_r $$','$$y $$','$$y_r $$','FontSize',12,'Interpreter','Latex','FontWeight','bold','Color','b','Location','southeast'))
legend('boxoff')
title('Position (m)','FontSize',12,'FontWeight','bold','FontName','Times New Roman','Color','b')
