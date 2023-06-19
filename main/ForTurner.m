% Main Script **********************
% Asigning robot specification
param.g = 9.8;
param.d = 0.08;     % ball diameter
param.r = 0.3;      % spring rest length/ chi0
param.m = 3;        % ball mass
param.k = 5000;     % vitual spring eleastic constant
param.b = 6;        % damping constant
param.N = 20;        % damping constant multiplication during stance phase
param.vb = 2*1*sqrt(param.k*param.m);
param.thrusttime = 0.005;  % duration of thrust
param.l1 = 0.15;
param.l2 = 0.3;


% Initial position Q0 = [q dq]
Q0 = [1 0];
tbegin = 0;
tfinal = 5;
phase = 4;

% ODE solving
T = tbegin;
Q = Q0;
while T(end) < tfinal
    if phase == 1
        % touch down, into stance phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventLiftOff(t, Q, param)));
        phase = 4;
%     elseif phase == 2
%         % stance phase, injecting thrust
%         [Ttemp, Qtemp] = ...
%             ode45(@(t, Q)EOMSpringThrust(t, Q, param),...
%             [tbegin, tbegin + param.thrusttime], Q0);
%         phase = 3;
%     elseif phase == 3
%         % stance phase without thrust
%         [Ttemp, Qtemp, te, Qe, ie] = ...
%             ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
%             odeset('Events',@(t, Q)EventLiftOff(t, Q, param)));
%         phase = 4;
    elseif phase == 4
        % lift off, into fly phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMFlight(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventTouchDown(t, Q, param)));
        phase = 1;
    end
    nT= length(Ttemp);
    T = [T; Ttemp(2:nT)];
    Q = [Q; Qtemp(2:nT,:)];
    Q0 = Qtemp(nT,:);
    tbegin = Ttemp(nT);
end

T2 = [0: 1e-2: tfinal];
Q2 = interp1(T, Q, T2);
% Find Inverse Kinematics MQ
xi = fivebarIK(Q2 , param);
% Simulation Visualization
Run5BarHopperSimulation(T2, Q2, xi, param, 'none')

% Equation of motions ****************
function dQ = EOMFlight(t, Q, param)
    % Describe the system equation of motion
    % Flight phase (from liftoff to touchdown)
    % Reminder: the state Q = [q dq]; state output dQ = [dq ddq]
    % Edit the following template
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.g;
end
function dQ = EOMStance(t, Q, param)
    % Stance phase (from touchdown to liftoff)
    tau = -param.vb/param.m*Q(2);
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.k/param.m*(Q(1)-param.r)-param.N*param.b/param.m*Q(2)-param.g+tau;
end
% function dQ = EOMSpringThrust(t,Q,param)
%     % Thrust phase
%     dQ = zeros(2,1);
%     dQ(1) = Q(2);
%     dQ(2) = -param.N*param.k/param.m*(Q(1)-param.r)-param.b/param.m*Q(2)-param.g;
% end

% Event function *********************
function [value,isterminal,direction] = EventTouchDown(t, Q, param)
    % Locate the gaurd (where the function turns 0) at value
    % determine the direction of the event
    % stop once the event happened with isterminal
    % Edit the following template
    value = Q(1)-param.r;     % event point: touch down
    direction = -1;           % direction
    isterminal = 1;           % stop the integration
end
function [value,isterminal,direction] = EventLiftOff(t, Q, param)
    value = Q(1)-param.r;     % event point: lift off
    direction = 1;            % direction
    isterminal = 1;           % stop the integration
end
% function [value,isterminal,direction] = EventBottom(t, Q, param)
%     value = Q(2);             % event point: bottom point
%     direction = 1;            % direction
%     isterminal = 1;           % stop the integration
% end

% Find Inverse Kinematics ****************
function xi = fivebarIK(Q , param)
    Q(Q(:,1)>param.r)=param.r;
    xi = acos((-param.l1^2+param.l2^2-Q(:,1).^2)./(2*param.l1.*Q(:,1)));
end

% Visualization *****************************
function Run5BarHopperSimulation(T, Q, MQ, param, savefilename)
tfinal = T(end);
d = param.d;
r = param.r;
l1= param.l1;
% Coordinate of COMs and indices
ox = 0;
oy = 0;
px = 0;
py = Q(:,1);
vy = Q(:,2);

% Color map for identification
colorblue = [0 0.447 0.741];
colorred = [0.85 0.325 0.098];
coloryellow = [0.9290 0.6940 0.1250];
colorgreen = [0.4660 0.6740 0.1880];

% Plot the first graph, then use data update to creat animation
figure('Name','Bouncing Ball');
% Setting figure
set(gcf, 'Position',  [10 10 1000 500]); % [left bottom width height]
    
% Motion position graph
% subplot position [left bottom width height]
subplot('Position',  [0.05 0.2 0.3 0.6]);
    % Visualizing the frame and coordinates
    rectangle('Position',[-1,-0.2,2,0.2]*max(py),...
        'FaceColor',[0.7,0.7,0.7],'EdgeColor',[0.6,0.6,0.6],'LineWidth',2)
    hold on;
    xcord = quiver(0,0,0.2,0,'LineWidth',2,'Color','k'); hold on;
    ycord = quiver(0,0,0,0.2,'LineWidth',2,'Color','k'); hold on;
    hold on;
    % Schematic Plot
    ball = rectangle('Position',[px-d,py(1)-d,2*d,2*d],...
        'Curvature',[1 1],'FaceColor',colorblue,...
        'EdgeColor',colorblue*0.7,'LineWidth',2);
%     spring = plot([px px px+d px-d px+d px-d ...
%         px+d px-d px+d px-d px px],...
%         [py(1) py(1)-0.3*r py(1)-0.325*r py(1)-0.375*r ...
%         py(1)-0.425*r py(1)-0.475*r py(1)-0.525*r py(1)-0.575*r ...
%         py(1)-0.625*r py(1)-0.675*r py(1)-0.7*r py(1)-r],...
%         '-','LineWidth',2,'Color','k');
    linkage = plot([px px-l1*sin(MQ(1)) px px+l1*sin(MQ(1)) px],...
        [py(1) py(1)+l1*cos(MQ(1)) py(1)-r py(1)+l1*cos(MQ(1)) py(1)],...
        '-','LineWidth',4,'Color','k');
    toe = rectangle('Position',[px-0.1*d,py(1)-r-0.1*d,0.2*d,0.2*d],...
        'Curvature',[1 1],'FaceColor',colorblue,...
        'EdgeColor',[0 0.447 0.741]*0.7,'LineWidth',2);
    hold on;
    % Vectors; quiver(x,y,u,v,scale)
    a1 = quiver(ox-r,oy,0,py(1),0,'LineWidth',3,'Color',colorred);
    hline = plot([ox-0.1*r ox-1.1*r],[py(1) py(1)],':',...
        'LineWidth',2,'Color',colorred);
    mangle = plot(px+0.5*d*sin(linspace(0,MQ(1))),...
        py(1)+0.5*d*cos(linspace(0,MQ(1))),...
        'LineWidth',3,'Color',coloryellow);
    vline = plot([ox ox],[py(1)+0.1*d py(1)+1.5*d],':',...
        'LineWidth',2,'Color',coloryellow);
    hold off;
    title('Schematic Motion');
    legend([a1,mangle(1)],'Position','Motor Angle')
    text(0.15,0.05,'x')
    text(0.05,0.15,'y')
    axis([-0.7*max(py),0.7*max(py),-0.2*max(py),1.2*max(py)]);
    xlabel('x (m)');ylabel('y (m)');

% Position graph
subplot('Position',  [0.4 0.56 0.5 0.24]);
    p2 = plot(T(1:1),py(1:1),'LineWidth',2,'Color',colorred);
    m2 = line(T(1),py(1),'Marker', 'o','LineWidth',4,'Color',colorred);
    title('Position Graph');
    axis([0,tfinal,-0.2*max(py),1.2*max(py)])
    grid on
    xlabel('time (sec)');ylabel('position (m)');
    
% Motor Input graph
subplot('Position',  [0.4 0.2 0.5 0.24]);
    p3 = plot(T(1:1),MQ(1:1),'LineWidth',2,'Color',coloryellow);
    m3 = line(T(1),MQ(1),'Marker','o','LineWidth',4,'Color',coloryellow);
    title('Motor Input Graph');
    axis([0,tfinal,0,pi])
    grid on
    xlabel('time (sec)');ylabel('Motor angle (rad)');

% Animating using Loops (Smart updating for efficency)
% http://web.mit.edu/8.13/matlab/MatlabTraining_IAP_2012/...
%     AGV/DemoFiles/ScriptFiles/html/Part3_Animation.html

% Get figure size
pos = get(gcf, 'Position');
width = pos(3);
height = pos(4);

% Preallocate
mov = zeros(height, width, 1, length(T), 'uint8');

for id = 1:length(T)
    % update XData and YData
    newr = r;
    if py(id)-r < 0
        newr = py(id);
    end
    set(ball , 'Position',[px-d,py(id)-d,2*d,2*d]);
%     set(spring, 'YData',[py(id) py(id)-0.35*newr py(id)-0.375*newr py(id)-0.425*newr ...
%         py(id)-0.475*newr py(id)-0.525*newr py(id)-0.575*newr py(id)-0.625*newr ...
%         py(id)-0.675*newr py(id)-0.725*newr py(id)-0.75*newr py(id)-newr]);
    set(linkage, 'XData', [px px-l1*sin(MQ(id)) px px+l1*sin(MQ(id)) px],...
        'YData',[py(id) py(id)+l1*cos(MQ(id)) py(id)-newr ...
        py(id)+l1*cos(MQ(id)) py(id)])
    set(toe  , 'Position',[px-0.1*d,py(id)-newr-0.1*d,0.2*d,0.2*d]);
    set(a1   , 'VData', py(id));
    set(hline, 'YData', [py(id) py(id)]);
    set(mangle, 'XData', px+0.5*d*sin(linspace(0,MQ(id))),...
        'YData', py(id)+0.5*d*cos(linspace(0,MQ(id))));
    set(vline, 'YData', [py(id)+0.1*d py(id)+1.5*d]);
    set(p2(1), 'XData', T(1:id)  , 'YData', py(1:id));
    set(m2(1), 'XData', T(id)    , 'YData', py(id));
    set(p3(1), 'XData', T(1:id)  , 'YData', MQ(1:id));
    set(m3(1), 'XData', T(id)    , 'YData', MQ(id));
    drawnow;

    if ~strcmpi(savefilename,'none')
        % Save as .gif
        f = getframe(gcf);
        img =  frame2im(f);
        [img,cmap] = rgb2ind(img,256);
        if id == 1
            imwrite(img,cmap,strcat(savefilename,'.gif'),'gif',...
                'LoopCount',Inf,'DelayTime',0);
        else
            imwrite(img,cmap,strcat(savefilename,'.gif'),'gif',...
                'WriteMode','append','DelayTime',0);
        end
    end
end
end