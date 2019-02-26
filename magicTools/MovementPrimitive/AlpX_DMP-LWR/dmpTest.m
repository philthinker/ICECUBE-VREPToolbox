%test dmp training func
% Alp Burak Pehlivan - A part of M.Sc. Thesis
% http://alpx.io/

clear all
% true for loading previously saved data
% false for mouse drawing
saved=true;

if saved==true
    load saveddata; % data file name (.mat)

else
    [x,y,vx,vy,ax,ay,nt,ttm,posu,times] = getUserTraj(0.01, 123);
    saveddata.x=x;
    saveddata.y=y;
    saveddata.vx=vx;
    saveddata.vy=vy;
    saveddata.ax=ax;
    saveddata.ay=ay;
    saveddata.times=times;
%     save('saveddata.mat','saveddata') 
    save('saveddata1.mat','saveddata') % you can change the file name.
end


% start time is initialized to zero! important for s
saveddata.times=saveddata.times-saveddata.times(1);
     
par=0;
par.ng=30;

% width=h(i)
% r=exp(-h(i)*(s-c(i))^2);
h=1;  % this is the Gaussian Width.
par.h=ones(1,par.ng)*(h);


par.s=1;
% par.as=4;
par.as=1;

par.K=1;
par.D=1;

r=dmpTrain(saveddata, par); 
r.penWidth1=2;
r.penWidth2=2;

result=dmpReplay(r);

result.options=[14];
% result.options=[1 3 14 31 5 7];
% result.options=[1 3 5 7 14];

% Blue: Original Data
% Red: DMP Replay
res=dmpPlot(saveddata, result);

% average error

xpos_error=mean(abs((saveddata.x-res.y_xr)))
ypos_error=mean(abs((saveddata.y-res.y_yr)))

xvel_error=mean(abs((saveddata.vx-res.yd_xr)))
yvel_error=mean(abs((saveddata.vy-res.yd_yr)))

xacc_error=mean(abs((saveddata.ax-res.ydd_xr)))
yacc_error=mean(abs((saveddata.ay-res.ydd_yr)))

totalPosition_error=xpos_error+ypos_error

% Here is the list of available plot options:
% any(r.options==1) original data x and replay
% any(r.options==2) original data x and replay with zero crossings of ftarget
% any(r.options==3) original data y and replay
% any(r.options==4) original data y and replay with zero crossings of ftarget
% any(r.options==5) ftarget x
% any(r.options==6) ftarget x with zero crossings
% any(r.options==7) ftarget y
% any(r.options==8) ftarget y with zero crossings
% any(r.options==9) wx
% any(r.options==10) wy
% any(r.options==11) xy data and with zero crossings of ftarget x
% any(r.options==12) xy data and with zero crossings of ftarget y
% any(r.options==13) phase variable s
% any(r.options==14) xy data and replay
% any(r.options==15) x pos replay error
% any(r.options==16) y pos replay error
% any(r.options==17) total pos replay error
% any(r.options==18) x vel replay error
% any(r.options==19) y vel replay error
% any(r.options==20) x acc replay error
% any(r.options==21) y acc replay error
% any(r.options==22) x pos
% any(r.options==23) y pos
% any(r.options==24) x vel
% any(r.options==25) y vel
% any(r.options==26) x acc
% any(r.options==27) y acc
% any(r.options==28) original data only
% any(r.options==29) ftargetx
% any(r.options==30) ftargetx
% any(r.options==31) s time

