function h = plot2DArrow(pos,dir,col,lnw,sz,alpha)
% Simple display of a 2D arrow
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Rozo16Frontiers,
%   author="Rozo, L. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
%   title="Learning Controllers for Reactive and Proactive Behaviors in Human-Robot Collaboration",
%   journal="Frontiers in Robotics and {AI}",
%   year="2016",
%   month="June",
%   volume="3",
%   number="30",
%   pages="1--11",
%   doi="10.3389/frobt.2016.00030"
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


if nargin<6
	alpha = 1;
end
if nargin<5
	sz = 1E-1;
end
if nargin<4
	lnw = 2;
end
if nargin<3
	col = [0,0,0];
end

msh = pos;
pos = pos+dir;
if norm(dir)>sz
  d = dir/norm(dir);
  prp = [d(2); -d(1)];
  d = d*sz;
  prp = prp*sz;
  msh = [msh pos-d pos-d-prp/2 pos pos-d+prp/2 pos-d msh];
  h = patch(msh(1,:),msh(2,:),col,'edgecolor',col,'linewidth',lnw,'edgealpha',alpha,'facealpha',alpha); 
end
