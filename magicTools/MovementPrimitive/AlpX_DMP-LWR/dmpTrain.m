% dmp train function for 2D only
% Alp Burak Pehlivan - A part of M.Sc. Thesis
% http://alpx.io/
function r=dmpTrain(data, par)

len=length(data.times);

x=data.x;
dx=data.vx;
ddx=data.ax;
dtx=diff(data.times);
dtx=dtx(1);
x0=x(1);
gx=x(end);

y=data.y;
dy=data.vy;
ddy=data.ay;
% dty=diff(data.times);
% dty=dty(1);
dty=dtx;
y0=y(1);
gy=y(end);


% number of gaussian basis functions
if ~isfield(par, 'ng')
    ng=len;
    par.ng=ng;
else
    ng=par.ng;
end

% decay of s phase var
if ~isfield(par, 'as')
    as=4;
    par.as=as;
else
    as=par.as;
end

% init of phase var
if ~isfield(par, 's')
    s=1;
    par.s=s;
else
    s=par.s;
end

% time scaling constant
if ~isfield(par, 'tau')
    % % tau should be duration of data
    % Miguel Prada and Anthony Remazeilles* Dynamic Movement Primitives for Human Robot interaction
%     tau=max(data.times)-min(data.times);
    tau=max(data.times);
    par.tau=tau;
else
    tau=par.tau;
end


% dynamical system parameters
% alpha*beta=K   beta=(K-D)/tau ?
% dynamical system
% tau*tau*ydd = K*(g-y)-D*yd+(g-y0)*f;
% tau*yd=yd;
if ~isfield(par, 'K')
    K=5;
    par.K=K;
else
    K=par.K;
end
% dynamical system parameters
if ~isfield(par, 'D')
    D=5;
    par.D=D;
else
    D=par.D;
end

% widths of gaussians
if ~isfield(par, 'h')
    % widths of gaussians. same but maybe adjustable according to the movement
    % maybe? check this later.
    h=ones(1,ng)*1; 
    par.h=h;
else
    h=par.h;
end



% ftarget calculation
stime=[];
sE_x=[];
sE_y=[];

for i=1:length(data.times)
    t=data.times(i);
    s=exp((-1*as*t)/tau);
    stime=[stime s];
    % fdemonstration=ftarget in time
    % glatzMaster page 9 . formula 2.4
    ftarget_x(i)= (-1*K*(gx-x(i))+D*dx(i)+tau*ddx(i))/(gx-x0);
    ftarget_y(i)= (-1*K*(gy-y(i))+D*dy(i)+tau*ddy(i))/(gy-y0);
    % ijspeert nc2013 page 343
    sE_x=[sE_x; s*(gx-x0)];
    sE_y=[sE_y; s*(gy-y0)];
end


% centers of gaussian are placed even in s time
if ~isfield(par, 'c')
    % gaussian centers are distributed evenly in s axis.
    incr=(max(stime)-min(stime))/(ng-1);
    c=min(stime):incr:max(stime);
    lrc=fliplr(c);
    ctime=(-1*tau*log(lrc))/as;
    d=diff(c);
    c=c/d(1); % normalize for exp correctness
    par.c=c;
else
    c=par.c;
end
    

% Regression 
for i=1:ng
    psV_x=[];   
    psV_y=[];   
    for j=1:length(data.times)
          psV_x=[psV_x psiF(h,c,stime(j)/d(1),i)];
          psV_y=[psV_y psiF(h,c,stime(j)/d(1),i)];
    end
%     L(i)={diag(psV)}; * Locally Weighted Learning
    w_x(i)=(transpose(sE_x)*diag(psV_x)*transpose(ftarget_x))/(transpose(sE_x)*diag(psV_x)*sE_x);
    w_y(i)=(transpose(sE_y)*diag(psV_y)*transpose(ftarget_y))/(transpose(sE_y)*diag(psV_y)*sE_y);
end


    r=par;
    r.len=len;
    r.times=data.times;
    r.stime=stime;
    r.ftarget_x=ftarget_x;
    r.ftarget_y=ftarget_y;
    r.w_x=w_x;
    r.w_y=w_y;
    r.x0=x0;
    r.y0=y0;
    r.gx=gx;
    r.gy=gy;
    r.d1=d(1);
    r.dt=dtx;
    r.ctime=ctime;
end