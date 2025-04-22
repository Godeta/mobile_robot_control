%Simuler la trajectoire d’un robot mobile dans un plan 2D, piloté par des
%vitesses linéaire et angulaire, pour qu’il suive une trajectoire de référence.
clear all;clc;close all;
%%-------------------------------------------------------------------------
his.dt=[];dt=0.05;t = 0:dt:30;
x=0;y=0;theta=1.1071;% position initiale du robot, angle = atan2(dyr,dxr) = atan2((4*pi*cos(0))/15, (2*pi*cos(0))/15)
X=[x,y,theta];

Vref=0;        % vitesse lineaire m/s a t=0 
Omegaref=0.0;  % vitesse anguaire rad/s a t=0 

%-----------------------Trajectoire de Référence---------------------------
x_des=2*sin(((2*pi)/30)*t);
y_des=2*sin(((4*pi)/30)*t);
% --------------------variables a tracer à la fin--------------------------
his.X=[];his.theta=[];his.Vref=[];his.Omegaref=[];his.thetar=[];
% -----------------figure contient toutes les variables
f3=figure;
f3.Position = [50 75 1450 700];
subplot(5,3,[1,14]),
%------------------------------Espace de travail---------------------------
hold on;grid on;
rectangle('Position',2.5*[-1.25,-1.25,2.5,2.5],'FaceColor','white','EdgeColor','#3f3f3f','LineWidth',5),
% 
text(-0.5,-2.2,' Validation: Controle BO d''un robot mobile','Color','[0 0 0]','FontSize',12);
text(-0.5,-2.5,' Realisé par Arnaud','Color','[0 0 0]','FontSize',12);
text(-0.5,-2.8,' Ecole HEI-JUNIA-Chateauroux','Color','[0 0 0]','FontSize',12);
% ----------Plot points
plot(0, 0,'r*','LineWidth',15);

% ------------------Animation
hold on;grid on;
PLOT.RefTrajectory=line('XData',[],'YData',[],'LineStyle','-.','color','r','LineWidth',1);
PLOT.Robot=line('XData',[],'YData',[],'LineStyle','--','color','b','LineWidth',1);
xlabel('x (m)')
ylabel('y (m)')
grid on

%  dimensions du robot 
A.R_w =0.1950; % robot width/2
A.R_l=0.1950;  % robot length/2
A.a1 = [-A.R_l -A.R_w]';
A.b1 = [A.R_l -A.R_w]';
A.b2 = [A.R_l A.R_w]';
A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c];
pl=[];

A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotation robot
A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; %
A.P_robot=patch(A.P(1,:),A.P(2,:),'k');
A.P_robot.XData=A.Prot_trasl(1,:)';
A.P_robot.YData=A.Prot_trasl(2,:)';
axis(2.5*[-1.25 1.25 -1.25 1.25]);  



%------------------------------ Programme principal -----------------------


legend(' Départ' ,' Trajectoire' , ' deplacement' ,'robot' );
for i=1:length(x_des)
   tt=t(i);  
  Ref_vitesse=CalculeVitesseRobot(tt);
  Vref=Ref_vitesse(1); 
  Omegaref=Ref_vitesse(2);
    % Simulation numérique du robot( integration numérique) 
    X=Mouve_Robot(X,[Vref,Omegaref],dt);
    % --------------------Fin
   % ----- Enregistrement de données  sous forme de vecteurs
    
    his.X=[his.X X];
    his.theta=[his.theta theta]; % l'angle theta
    his.Vref=[his.Vref Vref];% la commande V (vitesse lineaire)
    his.Omegaref=[his.Omegaref Omegaref]; %la commande W (vitesse angulaire)
  
% tracer line du robot
   
    set(PLOT.RefTrajectory,'XData',x_des(1:i),'YData',y_des(1:i));
    set(PLOT.Robot,'XData',his.X(1,:),'YData',his.X(2,:))
    
    A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotation
    A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; % ajout d'un offset pour le centre du robot
    A.P_robot.XData=A.Prot_trasl(1,:)';
    A.P_robot.YData=A.Prot_trasl(2,:)';
    drawnow  
end


