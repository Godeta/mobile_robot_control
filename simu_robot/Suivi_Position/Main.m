clear all;clc;close all;


%%----------------------------Commande BF-------------------------

%Dans cette Simulation, pour une position de référence donnée (xr, yr):
%la loi de commande conçue est basé sur les cordonnées polaires 
%pour conduire le robot de sa configuration actuelle, à la position cible


%----- déclaration de variables intiale du robot
dt=0.05; % pas de calcul dt (modifier pour avoir une solution plus en moins exact)
t = 0:dt:8;% temps de simulation
x=0;y=0;theta=0;% Orientation et position initial du robot 
X=[x,y,theta];% vecteur d'etat initial

v=0;    % vitesse lineaire initiale m/s a t=0 calculer a (t=0)
w=0;  % vitesse anguaire initial  rad/s a t=0  calculer a (t=0)

%-----------------------Trajectoire de Référence---------------------------
%Arrivee (voir la position du carré( rouge, blue, jaune, vert sur CoppeliaSim)
% on commence par le rouge et ainsi de suit
xr = -1;% x désiré
yr = -1;% y désiré
thetar=pi/4;
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
plot(xr,yr,'ro','LineWidth',10);hold on;
text(xr-0.1,yr+0.2,'Position désirée','Color','red','FontSize',12);

%  dimensions du robot 
A.R_w =0.07; % robot width/2
A.R_l=0.07;  % robot length/2
A.a1 = [-A.R_l -A.R_w]';A.b1 = [A.R_l -A.R_w]';A.b2 = [A.R_l A.R_w]';A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c];pl=[];

A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotation robot
A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; %
A.P_robot=patch(A.P(1,:),A.P(2,:),'r');A.P_robot.XData=A.Prot_trasl(1,:)';A.P_robot.YData=A.Prot_trasl(2,:)';
 


%-----------------Connexion automatique MATLAB/CoppeliaSim----------------- 
% Standard
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



xp0 = 0;yp0 =0;zp0 = 1.9150e-02;
% Orientation
ap0 = 0;bp0 = 0;cp0 =0;
[retCod] = vrep.simxSetObjectPosition(clientID,rob1,-1,[xp0,yp0,zp0],vrep.simx_opmode_oneshot);
[retCod] = vrep.simxSetObjectOrientation(clientID,rob1,-1,[ap0,bp0,cp0],vrep.simx_opmode_oneshot);
%% ---------------------------------------------------------------------------------



for i=1:length(t)
            
%--------------------------------------------------------------------------
[retCod,Position] = vrep.simxGetObjectPosition(clientID,rob1,-1,vrep.simx_opmode_oneshot_wait);
[retCod,Orientation] = vrep.simxGetObjectOrientation(clientID,rob1,-1,vrep.simx_opmode_oneshot_wait);




x = Position(1,1);y = Position(1,2);theta = Orientation(1,3);
X=[x,y,theta];


% --------------La loi de commande basée sur les cordonné polaire----------
% Intervenir ICI
% --------------La loi de commande basée sur les cordonné polaire----------
% Calcul des erreurs en coordonnées polaires
p = sqrt((xr-x)^2 + (yr-y)^2);  % Distance euclidienne à la cible
alpha = atan2(yr-y, xr-x) - theta;  % Erreur d'angle par rapport à la cible
alpha = atan2(sin(alpha), cos(alpha));  % Normalisation de l'angle entre -pi et pi
B = alpha + theta;  % Angle absolu désiré
B = atan2(sin(B), cos(B));  % Normalisation

% Gains du contrôleur
kp = 1.5;  % Gain sur la distance (contrôle la vitesse linéaire)
ka = 8;    % Gain sur l'erreur d'angle (contrôle la vitesse angulaire)
kb = -1;   % Gain sur l'angle absolu désiré

% Calcul des commandes
v = kp * p;  % Vitesse linéaire
w = ka * alpha + kb * B;  % Vitesse angulaire

% Limitation des vitesses
v_max = 0.4;  % Vitesse linéaire maximale
w_max = 4;    % Vitesse angulaire maximale
v = max(min(v, v_max), -v_max);
w = max(min(w, w_max), -w_max);

% Si la distance est très petite, on arrête le robot
if p < 0.05
    v = 0;
    w = 0;
end

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
