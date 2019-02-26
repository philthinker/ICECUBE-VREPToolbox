function [x,y,vx,vy,ax,ay,nt,ttm,posu,times] = getUserTraj(dt, fig)
% Erhan Oztop July 31, 2013. 
% Interactivelt draw and return a 2D trajectory. Press the mouse button and
% draw. The function will return when the button is released.
% Use with [x,y,vx,vy,ax,ay,nt] = getUserTraj(dt, fig)
% dt:  period of data to be returned (e.g. 0.01)
% fig: is the figure where drawing will be made (e.g. 1)
% If no parameters are entered it will default to dt=0.01, fig=123
% Returns: x,y as posions in x,y coordinates. vx, vy as velocities, 
% ax, ay: accelarations, nt is the time line so that you can
% for example do plot(nt, vx)

% 12.10.2014 changes by Alp Burak Pehlivan
% Alp Burak Pehlivan - A part of M.Sc. Thesis
% http://alpx.io/

global keyDown keyUp
global mousePos;
global mouseState;
global tm;
global poss;


if (~exist('dt','var'))
    dt  = 0.01;
    fig = 123;
end;
tm =0;
%times=[0];
times=[];
poss=[];
tm_tick = 0.001;
tic;

figh = figure(fig); cla;
set(figh, 'units', 'normalized')
set(figh, 'Position', [0.2 0.35 0.7 0.5])
plot(0,0,'+'); hold on; 
axis([-5,5,-5,5]); 
drawnow;
set(figh,'WindowButtonDownFcn',{@mousePressLocalCallback ,'DOWN'});
set(figh,'WindowButtonUpFcn',{@mousePressLocalCallback ,'UP'});
set(figh,'WindowButtonMotionFcn',{@mousePressLocalCallback ,'MOVE'});

keyDown = [];
keyUp   = [];

mousePos   = [-1,-1];
mouseState = 0;  % depressed


t=.005;
iter=0;
chck=1;
oldcoords = 0;
coords = [-1,-1];
cx = []; 
cy = [];
ct = [];
drawing = 0;
while (coords(1)~=1),
    coords = mousePos;
    if (mouseState==1),
        drawing = 1;
        %coords;
        cx = [cx coords(1)];
        cy = [cy coords(2)];
        ct = [ct tm];
        
        % i fixed posu var. now it is giving correct vals. time dependant
        poss=[poss; coords(1) coords(2)];
        plot(coords(1),coords(2),'o'); 

        times=[times tm];
    end;
    if (drawing == 1 && mouseState==0)
        drawing = 0;
        plot(cx,cy,'r-');
        break; % ------------------- Here we break from drawing loop    <-------
    end;
     if (keyDown==113), 
        break; 
    end;
    pause(tm_tick);
    tm = tm + tm_tick;
end
%----------- drawing is done now calculation 

SMOOTHVAL = 11;
ct = ct - ct(1);
ct = ct/ct(end); % orig time duration of traj
x=cx;
y=cy;
mydt=diff(ct);
mydt=mydt(1);

plot(x,y,'g-');
plot(x,y,'m.');
title('green with magenta dots is the returned trajectory');

x = x - x(1); % this is making the start (0,0) normalization
vx = [0 diff(x)/mydt];
ax = [0 diff(vx)/mydt];

y = y - y(1); % this is making the start (0,0) normalization
vy = [0 diff(y)/mydt];
ay = [0 diff(vy)/mydt];

nt = ct;

ttm=tm;
posu=poss;
%%

% Draws some of the returned parameters
% figure(2); clf;
% subplot(2,2,1);
% plot(ct,cx,'b.');
% subplot(2,2,2);
% plot(ct,cy,'b.');
% subplot(2,2,3);
% plot(myt1,x,'g.'); hold on;
% 
% plot(myt1,vx,'k.');
% 
% plot(myt1,ax,'c.');
% subplot(2,2,4);
% plot(myt1,y,'g.');


function mousePressLocalCallback(varargin)
global mouseState;
global mousePos;
global poss;

s = varargin{3};
if (strcmp(s,'UP')),
    mouseState = 0;
end;
if (strcmp(s,'DOWN')),
    mouseState = 1;
end;
if (strcmp(s,'MOVE') || strcmp(s,'DOWN')),

    C = get (gca, 'CurrentPoint');  
    t = toc;
    mousePos=[C(1,:), t];
%     if mousePos(1)~=-1 && mousePos(2)~=-1,
%         poss=[poss; mousePos];
%     end
end;