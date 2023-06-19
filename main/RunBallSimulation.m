% Visualization *****************************
function RunBallSimulation(T, Q, param, savefilename)
tfinal = T(end);
r = param.r;
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
% Setting figure [left bottom width height]
vidwidth = 1000;
vidheight = 500;
set(gcf, 'Position', [10 10 vidwidth vidheight]);
    
% Motion position graph
% Subplot position [left bottom width height]
subplot('Position',  [0.05 0.2 0.3 0.6]);
    % Visualizing the frame and coordinates
    rectangle('Position',[-1,-0.2,2,0.2]*max(py),...
        'FaceColor',[0.7,0.7,0.7],'EdgeColor',[0.6,0.6,0.6],'LineWidth',2)
    hold on;
    xcord = quiver(0,0,0.2,0,'LineWidth',2,'Color','k'); hold on;
    ycord = quiver(0,0,0,0.2,'LineWidth',2,'Color','k'); hold on;
    hold on;
    % Schematic Plot
    ball = rectangle('Position',[px-r,py(1)-r,2*r,2*r],...
        'Curvature',[1 1],'FaceColor',colorblue,...
        'EdgeColor',[0 0.447 0.741]*0.7,'LineWidth',2);
    hold on;
    % Vectors; quiver(x,y,u,v,scale)
    a1 = quiver(ox-4*r,oy,0,py(1),0,'LineWidth',3,'Color',colorred);
    a2 = quiver(px,py(1),vy(1),0,0.1,'LineWidth',3,'Color',coloryellow);
    hline = plot([ox-0.1*r ox-4.5*r],[py(1) py(1)],':',...
        'LineWidth',2,'Color',colorred);
    hold off;
    title('Schematic Motion');
    legend([a1,a2],'COM Position','COM Velocity');
    text(0.15,0.05,'x');
    text(0.05,0.15,'y');
    axis([-0.7*max(py),0.7*max(py),-0.2*max(py),1.2*max(py)]);
    xlabel('x (m)');ylabel('y (m)');

% Position graph
subplot('Position',  [0.4 0.56 0.2 0.24]);
    p2 = plot(T(1:1),py(1:1),'LineWidth',2,'Color',colorred);
    m2 = line(T(1),py(1),'Marker', 'o','LineWidth',4,'Color',colorred);
    title('Position Graph');
    axis([0,tfinal,-0.2*max(py),1.2*max(py)]);
    grid on;
    xlabel('time (sec)');ylabel('position (m)');
    
% Velocity graph
subplot('Position',  [0.4 0.2 0.2 0.24]);
    p3 = plot(T(1:1),vy(1:1),'LineWidth',2,'Color',coloryellow);
    m3 = line(T(1),vy(1),'Marker','o','LineWidth',4,'Color',coloryellow);
    title('Velocity Graph');
    axis([0,tfinal,-1.5*max(vy),1.5*max(vy)]);
    grid on;
    xlabel('time (sec)');ylabel('velocity (m/s)');
    
% Orbit in State Space    
subplot('Position',  [0.65 0.2 0.3 0.6]);
    p4 = plot(py(1:1),vy(1:1),'LineWidth',2,'Color', colorgreen);
    m4 = line(py(1),vy(1),'Marker', 'o','LineWidth',4,'Color', colorgreen);
    title('Orbit in State Space');
    axis([-0.2*max(py),1.2*max(py),-1.5*max(vy),1.5*max(vy)]);
    grid on; grid minor;
    xlabel('position (m)');ylabel('velocity (m/s)');

% Animating using Loops (Smart updating for efficency)
% http://web.mit.edu/8.13/matlab/MatlabTraining_IAP_2012/...
%     AGV/DemoFiles/ScriptFiles/html/Part3_Animation.html

% Preallocate
mov = zeros(vidheight, vidwidth, 1, length(T), 'uint8');

for id = 1:length(T)
    % update XData and YData
    newh = r;
    neww = r;
    if py(id)-r < 0
        newh = py(id);
        neww = 1.5*r - 0.5*py(id);
    end
    set(ball, 'Position',[px-neww,py(id)-newh,2*neww,2*newh]);
    set(a1   , 'VData', py(id));
    set(a2   , 'YData', py(id)   , 'VData', vy(id));
    set(hline, 'YData', [py(id) py(id)]);
    set(p2(1), 'XData', T(1:id)  , 'YData', py(1:id));
    set(m2(1), 'XData', T(id)    , 'YData', py(id));
    set(p3(1), 'XData', T(1:id)  , 'YData', vy(1:id));
    set(m3(1), 'XData', T(id)    , 'YData', vy(id));
    set(p4(1), 'XData', py(1:id) , 'YData', vy(1:id));
    set(m4(1), 'XData', py(id)   , 'YData', vy(id));
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