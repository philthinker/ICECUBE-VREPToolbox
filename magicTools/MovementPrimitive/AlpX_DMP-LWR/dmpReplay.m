% dmpReplay
% Alp Burak Pehlivan - A part of M.Sc. Thesis
% http://alpx.io/

function result=dmpReplay(r)

% set new x0 goal
if isfield(r, 'sx0')
    r.x0=r.sx0;
end

% set new gy0 goal
if isfield(r, 'sy0')
    r.y0=r.sy0;
end

% set new gx goal
if isfield(r, 'sgx')
    r.gx=r.sgx;
end

% set new gy goal
if isfield(r, 'sgy')
    r.gy=r.sgy;
end

% from now on it dmp replay and plots
f_replay_x=[];
fr_x_zeros=[];

f_replay_y=[];
fr_y_zeros=[];

ydd_x_r=0;
yd_x_r=0;
y_x_r=r.x0;
dtx=r.dt;

ydd_y_r=0;
yd_y_r=0;
y_y_r=r.y0;
dty=dtx;


% r.gx=r.gx*3/2;  % replay da goal'u scale edebilirim 


for j=1:length(r.times)
    psum_x=0; 
    pdiv_x=0;
    psum_y=0; 
    pdiv_y=0;
    for i=1:r.ng
        % I am dividing the stime with d1=which is the dt of movement
        % therefore I normalize the centers of gaussians 
        % which were evenly distributed in time before.
        % now they are evenly place with 1 sec time diff.
        psum_x=psum_x+psiF(r.h, r.c, r.stime(j)/r.d1,i)*r.w_x(i);
        pdiv_x=pdiv_x+psiF(r.h, r.c, r.stime(j)/r.d1,i);

        psum_y=psum_y+psiF(r.h, r.c, r.stime(j)/r.d1,i)*r.w_y(i);
        pdiv_y=pdiv_y+psiF(r.h, r.c, r.stime(j)/r.d1,i);
    end
% replay of f with trained weights
% ijspeert nc2013 page 333 formula 2.3
    f_replay_x(j)=(psum_x/pdiv_x)*r.stime(j)*(r.gx-r.x0);
    %? I had to put -1 here in saveddata. otherwise y axis was reverse
    f_replay_y(j)=(psum_y/pdiv_y)*r.stime(j)*(r.gy-r.y0); 

    %detect zero crossing. sign change and list them
    if(j>1)
       if sign(f_replay_x(j-1))~=sign(f_replay_x(j))
           fr_x_zeros=[fr_x_zeros j-1];
       end
       
       if sign(f_replay_y(j-1))~=sign(f_replay_y(j))
           fr_y_zeros=[fr_y_zeros j-1];
       end
    end
    
    % We are calculating accelleration with ftarget replays on the system
    % then calculate the velocity and accelleration.
    % glatzMaster page 8 / eq 2.1   
    % tau*ydd= K(g-x)-Dv+(g-x0)f
    ydd_x_r=(r.K*(r.gx-y_x_r)-(r.D*yd_x_r)+(r.gx-r.x0)*f_replay_x(j))/r.tau;
    yd_x_r= yd_x_r+ (ydd_x_r*dtx)/r.tau;
    y_x_r= y_x_r+ (yd_x_r*dtx)/r.tau;
    
    ydd_xr(j)=ydd_x_r;
    yd_xr(j)=yd_x_r;
    y_xr(j)=y_x_r;
    
    %%%%%%%
    ydd_y_r=(r.K*(r.gy-y_y_r)-(r.D*yd_y_r)+(r.gy-r.y0)*f_replay_y(j))/r.tau;
    yd_y_r= yd_y_r + (ydd_y_r*dty)/r.tau;
    y_y_r= y_y_r + (yd_y_r*dty)/r.tau;
    
    ydd_yr(j)=ydd_y_r;
    yd_yr(j)=yd_y_r;
    y_yr(j)=y_y_r;
end

result=r;
result.ydd_yr=ydd_yr;
result.yd_yr=yd_yr;
result.y_yr=y_yr;
result.ydd_xr=ydd_xr;
result.yd_xr=yd_xr;
result.y_xr=y_xr;
result.fr_x_zeros=fr_x_zeros;
result.fr_y_zeros=fr_y_zeros;

result.f_replay_x=f_replay_x;
result.f_replay_y=f_replay_y;


end