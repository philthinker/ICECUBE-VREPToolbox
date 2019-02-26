% plot dmp results
% Alp Burak Pehlivan - A part of M.Sc. Thesis
% http://alpx.io/
function res=dmpplot(data, r)


colors='rgbycmrgbycmrgbycmrgbycmrgbycmrgbycm';


if ~isfield(r, 'penWidth1')
    penWidth1=1;
    r.penWidth1=1;
else
    penWidth1=r.penWidth1;
end

if ~isfield(r, 'penWidth2')
    penWidth2=1;
    r.penWidth2=1;
else
    penWidth2=r.penWidth2;
end


% find zero crossing sign changes in weights
fp=find(r.w_x>0);
fplist=find(diff(fp)>1);
fm=find(r.w_x<0);
fmlist=find(diff(fm)>1);
r.w_x_zeros=[1 sort([fplist fmlist]) length(r.w_x)];
r.w_x_zeros=unique(r.w_x_zeros);

r.wxzeros_time = floor(r.w_x_zeros*length(data.x)/r.ng);

% for wy
fp=find(r.w_y>0);
fplist=find(diff(fp)>1);
fm=find(r.w_y<0);
fmlist=find(diff(fm)>1);
r.w_y_zeros=[1 sort([fplist fmlist]) length(r.w_y)];
r.w_y_zeros=unique(r.w_y_zeros);

r.wyzeros_time = floor(r.w_y_zeros*length(data.y)/r.ng);

r.fr_x_zeros=[1 r.fr_x_zeros length(r.y_xr)];
r.fr_x_zeros=unique(r.fr_x_zeros);

r.fr_y_zeros=[1 r.fr_y_zeros length(r.y_xr)];
r.fr_y_zeros=unique(r.fr_y_zeros);

if true
    
if any(r.options==1)
    %original data x and replay
    figure;
    plot(data.times, data.x,'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('data.x');
    hold on
    plot(data.times, r.y_xr, 'r','LineWidth', penWidth2);
    xlabel('time (sec)');
    ylabel('x replay from DMP');
end


if any(r.options==2)
    %original data x and replay with zero crossings of ftarget x
    figure;
    plot(data.times, data.x, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('data.x');
    hold on
    plot(data.times, r.y_xr, 'r', 'LineWidth',penWidth2);
    xlabel('time (sec)');
    ylabel('data.x replay with zero crossings');
    hold on;
    for i=1:length(r.fr_x_zeros)-1
        plot(data.times( r.fr_x_zeros(i):r.fr_x_zeros(i+1) ), r.y_xr( r.fr_x_zeros(i):r.fr_x_zeros(i+1) ), colors(i), 'LineWidth', penWidth2);
        hold on
    end
end
    
if any(r.options==3)
    %original data y and replay
    figure
    plot(data.times, data.y, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('data.y');
    hold on
    plot(data.times, r.y_yr, 'r', 'LineWidth',penWidth2);
    xlabel('time (sec)');
    ylabel('y replay from DMP');
end

if any(r.options==4)
    %original data y and replay with zero crossings of ftarget y
    figure
    plot(data.times, data.y, penWidth1);
    xlabel('time (sec)');
    ylabel('data.y');
    hold on
    plot(data.times, r.y_yr, 'r', penWidth2);
    xlabel('time (sec)');
    ylabel('data.y replay with zero crossings');
    hold on;
    for i=1:length(r.fr_y_zeros)-1
        plot(data.times( r.fr_y_zeros(i):r.fr_y_zeros(i+1) ), r.y_yr( r.fr_y_zeros(i):r.fr_y_zeros(i+1) ), colors(i), 'LineWidth', penWidth2);
        hold on
    end
end
    

if any(r.options==5)    
    % ftarget x 
    figure;
    plot(data.times, r.ftarget_x, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('ftarget x');
    hold on;
    plot(data.times, r.f_replay_x, 'r', 'LineWidth',penWidth2);
    xlabel('time (sec)');
    ylabel('ftarget_x fitted by DMP');
end


if any(r.options==6)    
    % ftarget x with zero crossings
    figure;
    plot(data.times, r.ftarget_x, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('ftarget x');
    hold on;
    plot(data.times, r.f_replay_x, 'r','LineWidth', penWidth2);
    xlabel('time (sec)');
    ylabel('ftarget x fitted from dmp with 0-cross');
    hold on;
    for i=1:length(r.fr_x_zeros)-1
        plot(data.times( r.fr_x_zeros(i):r.fr_x_zeros(i+1) ), r.f_replay_x( r.fr_x_zeros(i):r.fr_x_zeros(i+1) ), colors(i), 'LineWidth', penWidth2);
        hold on
    end
end
    
if any(r.options==7)
    % ftarget y
    figure
    plot(data.times, r.ftarget_y, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('ftarget y');
    hold on
    plot(data.times, r.f_replay_y, 'r', 'LineWidth',penWidth2);
    xlabel('time (sec)');
    ylabel('ftarget_y fitted by DMP');
end


if any(r.options==8)
    % ftarget y with zero crossings
    figure
    plot(data.times, r.ftarget_y, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('ftarget y');
    hold on
    plot(data.times, r.f_replay_y, 'r','LineWidth', penWidth2);
    xlabel('time (sec)');
    ylabel('ftarget y fitted from dmp with 0-cross');
    hold on;
    for i=1:length(r.fr_y_zeros)-1
        plot(data.times( r.fr_y_zeros(i):r.fr_y_zeros(i+1) ), r.f_replay_y( r.fr_y_zeros(i):r.fr_y_zeros(i+1) ), colors(i), 'LineWidth', penWidth2);
        hold on
    end
end
    
if any(r.options==9)
    % wx
    figure
    plot(r.w_x, 'LineWidth',penWidth1);
    xlabel('Gaussian Basis Function #');
    ylabel('wx');
end

if any(r.options==10)
    % wy
    figure
    plot(r.w_y, 'LineWidth',penWidth1);
    xlabel('Gaussian Basis Function #');
    ylabel('wy');
end

end

if any(r.options==11)
 % xy data and with zero crossings of ftarget x
    figure
    plot(data.x,data.y, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('data');
    hold on
    for i=1:length(r.fr_x_zeros)-1
        plot(r.y_xr( r.fr_x_zeros(i):r.fr_x_zeros(i+1) ), r.y_yr( r.fr_x_zeros(i):r.fr_x_zeros(i+1) ), colors(i), 'LineWidth', penWidth2);
        hold on
    end
    scatter( r.y_xr(r.wxzeros_time), r.y_yr(r.wxzeros_time),200,'rp','filled');
    scatter( r.y_xr(r.wyzeros_time), r.y_yr(r.wyzeros_time),200,'gp','filled');
    xlabel('time (sec)');
    ylabel('xy replay with zero crossing ftargetx');
end


if any(r.options==12)
 % xy data and with zero crossings of ftarget y
    figure
    plot(data.x,data.y,'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('data');
    hold on
    for i=1:length(r.fr_y_zeros)-1
        plot(r.y_xr( r.fr_y_zeros(i):r.fr_y_zeros(i+1) ), r.y_yr( r.fr_y_zeros(i):r.fr_y_zeros(i+1) ), colors(i), 'LineWidth', penWidth2);
        hold on
    end
    scatter( r.y_xr(r.wxzeros_time), r.y_yr(r.wxzeros_time),200,'rp','filled');
    scatter( r.y_xr(r.wyzeros_time), r.y_yr(r.wyzeros_time),200,'gp','filled');
    xlabel('time (sec)');
    ylabel('xy replay with zero crossing ftargety');
end

if any(r.options==13)
 % phase variable s
    figure
    plot(data.times,r.stime, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('s');
end

if any(r.options==14)
 % xy data and replay
    figure
    plot(data.x,data.y, 'LineWidth',penWidth1);
    xlabel('time (sec)');
    ylabel('data');
    hold on
    plot(r.y_xr, r.y_yr, 'r', 'LineWidth', penWidth2);
    xlabel('time (sec)');
    ylabel('xy replay from DMP');
end


if any(r.options==15)
 % x pos replay error
    figure
    plot(data.times, abs((data.x-r.y_xr)./data.x)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('x-DMP replay error %');
end


if any(r.options==16)
 % y pos replay error
    figure
    plot(data.times, abs((data.y-r.y_yr)./data.y)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('y-DMP replay error %');
end

if any(r.options==17)
 % total pos replay error
    figure
    plot(data.times, abs((data.y-r.y_yr)./data.y)*100 + abs((data.x-r.y_xr)/data.x)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('x and y position-DMP replay error %');
end


if any(r.options==18)
 % x vel replay error
    figure
    plot(data.times, abs((data.vx-r.yd_xr)./data.vx)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('v_x-DMP replay error %');
end

if any(r.options==19)
 % y vel replay error
    figure
    plot(data.times, abs((data.vy-r.yd_yr)./data.vy)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('v_y-DMP replay error %');
end


if any(r.options==20)
 % x acc replay error
    figure
    plot(data.times, abs((data.ax-r.ydd_xr)./data.ax)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('a_x-DMP replay error %');
end

if any(r.options==21)
 % y acc replay error
    figure
    plot(data.times,abs((data.ay-r.ydd_yr)./data.ay)*100, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('a_y-DMP replay error %');
end


if any(r.options==22)
 % x pos
    figure
    plot(data.times,data.x, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('x');
end

if any(r.options==23)
 % y pos
    figure
    plot(data.times,data.y, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('y');
end

if any(r.options==24)
 % xvel
    figure
    plot(data.times,data.vx, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('v_x');
end

if any(r.options==25)
 % yvel
    figure
    plot(data.times,data.vy, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('v_y');
end

if any(r.options==26)
 % x acc
    figure
    plot(data.times,data.ax, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('a_x');
end

if any(r.options==27)
 % y acc
    figure
    plot(data.times,data.ay, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('a_y');
end

if any(r.options==28)
 % orig data
    figure
    plot(data.x,data.y, 'b', 'LineWidth', penWidth1);
    xlabel('x');
    ylabel('y');
end

if any(r.options==29)
 % ftarget x
    figure
    plot(data.times,r.ftarget_x, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('ftarget_x');
end

if any(r.options==30)
 % ftarget y
    figure
    plot(data.times,r.ftarget_y, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('ftarget_y');
end

if any(r.options==31)
 % s
    figure
    plot(data.times,r.stime, 'b', 'LineWidth', penWidth1);
    xlabel('time (sec)');
    ylabel('s');
end

res=r;
end

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


     