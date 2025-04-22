close all;clear all;clc;
coppelia =1;

global  d r1 dt
% paramètres robot
%d = 0.1950; r1 = d/2; % wheel radius
d = 0.053; r1=d/2; L=0.021;

% --    Trajectoire    --
dt=0.1;
%position initiale
x=0;y=0;theta=0;
X=[x;y;theta];
tperiod = 2*[0 2.5 5 10 15 20 22.5 25];
t = 0:dt:tperiod(end);

%points = [0 0; 1 0; 1 1; -1 1;-1 -1; 1 -1; 1 0; 0 0];
%x_points = points(:,1);
%y_points = points(:,2);

%positions à atteindre
xr=[-1,1,1,-1]; 
yr=[-1,-1,1,1];

%interpolation
%xr=[0,-1,1,0.9,-1.1];
%yr=[0,-1,-0.9,1,1.1];
% temps1 = [0,1,2,3,4];
% new_t=0:0.1:max(temps1);
% xr=interp1(temps1,xr,new_t);
% yr=interp1(temps1,yr,new_t);

%trajectoire de cercle :
% xr = cos(t)
% yr = sin(t)

%ligne courbée :
%xr = t / max(t);       % ligne droite normalisée de 0 à 1
%yr = 0.5 * sin(xr * pi); % courbe en S

%trajectoire
%xr = 2*sin(((2*pi)/30)*t);
%yr = 2*sin(((4*pi)/30)*t);

%point
%xr = -1;
%yr = 1;

%point 
%xr = t*0+1
%yr = t*0+1
%tperiod = [10 15 20 25];
%x_des=interp1(tperiod,xr,t)

%interpolation
% xtest = 0:-pi/4:-1.5*pi; 
% vtest = sin(xtest);
% xqtest = 0:-pi/16:-1.5*pi;
% vq1 = interp1(xtest,vtest,xqtest);
% xr=vq1;
% yr=vq1;

%angle à atteindre
xr = 1
yr = 1
theta_goal = 1; % Par exemple, orienté vers le haut (90°)

Vref=0;        % max 0.5 m/s
Omegaref=0.0;    % max 3 rps
% --- Initialisation des variables PID ---
integral_p = 0;
previous_p = 0;
integral_alpha = 0;
previous_alpha = 0;

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
plot(xr, yr, 'r-.', 'LineWidth', 1.5); % trace de la trajectoire de référence

hold on;grid on;
PLOT.Robot=line('XData',[],'YData',[],'LineStyle','--','color','g','LineWidth',1);
xlabel('x (m)')
ylabel('y (m)')
%legend('Reference Trajectory','Acual')
grid on

%  robot dimensions
A.R_w =r1; % robot width/2
A.R_l=r1;   % robot length/2
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
if (coppelia >0)
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,1);
vrep.simxSynchronous(clientID,true);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
else 
    clientID = -1
end
connected = false;
if (clientID>-1)
connected = true;
disp('Connected to remote API server');
%% ---------------------------------start simulation-----------------------
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);   
%%
[~,Robot] = vrep.simxGetObjectHandle(clientID,'ePuck',vrep.simx_opmode_blocking);
[~,rightJoint] = vrep.simxGetObjectHandle(clientID,'ePuck_rightJoint',vrep.simx_opmode_blocking);
[~,leftJoint] = vrep.simxGetObjectHandle(clientID,'ePuck_leftJoint',vrep.simx_opmode_blocking);

[~,Position] = vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_streaming);
[~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_streaming);
[~,Position] = vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer);
[~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_buffer);

end



     
%affichage infos
legend("origine", "destination", "consigne", "robot");

    
    %boucle déplacement
    threshold = 0.1; % tolérance pour considérer le point atteint
    XX = [];
    YY = [];
for j = 1:length(xr)
    point_reached = false;
    while ~point_reached
        % Lire la position actuelle
        if (connected) 
            [~,Position] = vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_buffer);
            [~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_buffer);
            %if (xr < 0)
            Orientation = - Orientation; %inversé dans coppeliaSim
            %end
        else
            Position = X;
            Orientation = X;
            disp(X)
        end

        %% Variables d'état
        x = Position(1);
        y = Position(2);
        theta = Orientation(3); %+ pi/2; % + pi/2; % ou -pi/2 selon l'orientation initiale;
        X = [x, y, theta];

        %% Contrôle
        p = sqrt((xr(j)-x)^2 + (yr(j)-y)^2);
        alpha = atan2(yr(j)-y, xr(j)-x) - theta;
        alpha = AngleWrap(alpha);
        B = alpha + theta;
        B = AngleWrap(B);

        %% Contrôleur Proportionnel TP
%         
        kp = 2; %: distance euclidienne à la cible (erreur de position)
        ka = 15; %erreur d’orientation par rapport à la trajectoire (cap désiré - cap actuel)
        kb = -0.2; %	Gain sur l’angle absolu désiré (gamma ou y)

        v = kp * p;
        w = ka * alpha + kb * B;

        % --- Mise à jour des états PID ---
%         integral_p = integral_p + p * dt;
%         derivative_p = (p - previous_p) / dt;
%         previous_p = p;
% 
%         integral_alpha = integral_alpha + alpha * dt;
%         derivative_alpha = (alpha - previous_alpha) / dt;
%         previous_alpha = alpha;
% 
%         % --- Définition des gains PID ---
%         kp_lin = 2;    % gain proportionnel pour p
%         ki_lin = 0.1;  % gain intégral pour p
%         kd_lin = 0.05; % gain dérivé pour p
% 
%         kp_ang = 15;   % gain proportionnel pour alpha
%         ki_ang = 2;  % gain intégral pour alpha
%         kd_ang = 0.5;  % gain dérivé pour alpha
% 
%         % --- Calcul du contrôle PID ---
%         v = kp_lin * p + ki_lin * integral_p + kd_lin * derivative_p;
%         w = kp_ang * alpha + ki_ang * integral_alpha + kd_ang * derivative_alpha;

        %% Limitation
        v_max = 0.7;
        %w_max = 10;
        v = max(min(v, v_max), -v_max);
        %w = max(min(w, w_max), -w_max);

        %% Calcul des vitesses des roues
        V_r = (2*v + L*w) / (2*r1);
        V_l = (2*v - L*w) / (2*r1);
        V_r = single(V_r)
        V_l = single(V_l)
        
        %% Commande des moteurs ou simulation
        if (connected)
            vrep.simxSetJointTargetVelocity(clientID, rightJoint, V_r, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID, leftJoint, V_l, vrep.simx_opmode_blocking);
            vrep.simxSynchronousTrigger(clientID);
        else
            X(1) = X(1) + v * cos(X(3)) * dt;
            X(2) = X(2) + v * sin(X(3)) * dt;
            X(3) = X(3) + w * dt;
        end

        %% Historique + Affichage
        his.X = [his.X X];   
        XX(j) = X(1);
        YY(j) = X(2);    

        set(PLOT.Robot, 'XData', XX, 'YData', YY); 
        A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))] * A.P; 
        A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)];
        A.P_robot.XData = A.Prot_trasl(1,:)';
        A.P_robot.YData = A.Prot_trasl(2,:)';
        drawnow 
        %pause(dt)
        
        
        
        %% Condition d'arrêt
        if p < threshold
            point_reached = true;
            % Marquer le point atteint (en vert)
            plot(xr(j), yr(j), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
            text(xr(j), yr(j) + 0.05, sprintf('Point %d', j), 'Color', 'green', 'FontSize', 8);
        end  
    end
    
    % Atteindre orientation finale après le dernier point
        if j == length(xr)
            theta_threshold = 0.1; % tolérance en radians

            while abs(AngleWrap(theta - theta_goal)) > theta_threshold
                % Lire l'orientation actuelle
                if (connected)
                    [~,Orientation] = vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_buffer);
                    Orientation = - Orientation;
                    theta = Orientation(3);
                else
                    theta = X(3);
                end

                % Calcul de l'erreur angulaire
                theta_error = AngleWrap(theta_goal - theta);
                w = 2 * theta_error; % gain proportionnel
                v = 0; % Pas de déplacement linéaire

                % Calcul des vitesses des roues
                V_r = (2*v + L*w) / (2*r1);
                V_l = (2*v - L*w) / (2*r1);

                if (connected)
                    vrep.simxSetJointTargetVelocity(clientID, rightJoint, V_r, vrep.simx_opmode_blocking);
                    vrep.simxSetJointTargetVelocity(clientID, leftJoint, V_l, vrep.simx_opmode_blocking);
                    vrep.simxSynchronousTrigger(clientID);
                else
                    X(3) = X(3) + w * dt;
                end

                drawnow
            end
        end
end

[~]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);


%% ---------------------------------stop simulation------------------------   
    vrep.simxFinish(clientID);

vrep.delete(); % call the destructor!

%% ----- Tracer les résultats
