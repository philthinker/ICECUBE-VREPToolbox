function [p2] = plotArmLink(a1,d1,p1,sz,txt);
%
% This function plots an a link of the robotic arm in 2D space.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org

nbSegm = 50;

t1 = linspace(0,-pi,nbSegm/2);
t2 = linspace(pi,0,nbSegm/2);
xTmp(1,:) = [sz.*sin(t1) d1+sz.*sin(t2)];
xTmp(2,:) = [sz.*cos(t1) sz.*cos(t2)];
R = [cos(a1) -sin(a1); sin(a1) cos(a1)];
x = R*xTmp + repmat(p1,1,nbSegm);
p2 = R*[d1;0] + p1;
fill(x(1,:),x(2,:),[1 1 1]);
rectangle('Position',[p1(1)-sz*.4,p1(2)-sz*.4,sz*.8,sz*.8], 'Curvature',[1,1], ...
  'LineWidth',1,'LineStyle','-','facecolor',[1 1 1],'edgeColor',[0 0 0])
rectangle('Position',[p2(1)-sz*.4,p2(2)-sz*.4,sz*.8,sz*.8],'Curvature',[1,1], ...
  'LineWidth',1,'LineStyle','-','facecolor',[1 1 1],'edgeColor',[0 0 0])
ptxt = R*[d1*0.7;0.8] + p1;
text(ptxt(1),ptxt(2),txt);
