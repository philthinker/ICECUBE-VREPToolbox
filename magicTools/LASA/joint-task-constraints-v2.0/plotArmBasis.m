function plotArmBasis(p1,sz)
%
% This function plots the basis of the simulated robotic arm in 2D space.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org

nbSegm = 50;

t1 = linspace(0,pi,nbSegm-2);
xTmp(1,:) = [sz*1.5 sz.*1.5*cos(t1) -sz*1.5];
xTmp(2,:) = [-sz*1.2 sz.*1.5*sin(t1) -sz*1.2];
x = xTmp + repmat(p1,1,nbSegm);
fill(x(1,:),x(2,:),[1 1 1]);

xTmp2(1,:) = linspace(-sz*1.2,sz*1.2,5); 
xTmp2(2,:) = repmat(-sz*1.2,1,5);
x2 = xTmp2 + repmat(p1,1,5);
x3 = xTmp2 + repmat(p1+[-.2;-.4],1,5);
for i=1:5
  plot([x2(1,i) x3(1,i)], [x2(2,i) x3(2,i)], 'k-');
end
