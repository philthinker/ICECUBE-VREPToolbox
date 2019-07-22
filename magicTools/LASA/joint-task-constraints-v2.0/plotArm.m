function plotArm(a,d,p1)
%
% This function plots the simulated robotic arm in 2D space.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org

sz = .5;

plotArmBasis(p1,sz);
p2 = plotArmLink(a(1),d(1),p1,sz,'L1');
p3 = plotArmLink(a(2),d(2),p2,sz,'L2');
if length(a)==3
  p4 = plotArmLink(a(3),d(3),p3,sz,'L3');
end
