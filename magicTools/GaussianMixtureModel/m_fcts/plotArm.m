function h = plotArm(a, d, p, sz, facecolor, edgecolor)
% Display of a planar robot arm.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon15,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%   year="2015"
% }
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.


if nargin<4
	sz = .05;
end
if nargin<5
	facecolor = [.5,.5,.5];
end
if nargin<6
	edgecolor = [1,1,1];
end
if size(p,1)==2
	p = [p; -1];
end

h = plotArmBasis(p, sz, facecolor, edgecolor);
for i=1:length(a)
	[p, hTmp] = plotArmLink(sum(a(1:i)), d(i), p, sz, facecolor, edgecolor);
	h = [h hTmp];
end
