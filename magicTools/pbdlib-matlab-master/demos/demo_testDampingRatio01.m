function demo_testDampingRatio01
% Behaviors of critically damped systems and ideal underdamped systems.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon16JIST,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
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


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 100; %Number of datapoints
kp = 200; %Stiffness
kv = 2*kp^.5; %Damping for critically damped system
kv2 = (2*kp)^.5; %Damping for ideal underdamped system
xhat = 1; %Desired target
dt = 0.01; %Time step


%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Computation of tracking error for critically damped system
x=0; dx=0; e=0;
for t=1:nbData
	Data(:,t) = [x; dx]; %Log data
	ddx = kp*(xhat-x) - kv * dx; %Spring-damper system
	dx = dx + ddx*dt; %Update velocity
	x = x + dx*dt; %Update position
	e = e + (xhat-x); %Cumulated tracking error
end
disp(['Cumulated tracking error for critically damped system: ' num2str(e)]);

%Computation of tracking error for ideal underdamped system
x=0; dx=0; e=0;
for t=1:nbData
	Data2(:,t) = [x; dx]; %Log data
	ddx = kp*(xhat-x) - kv2 * dx; %Spring-damper system
	dx = dx + ddx*dt; %Update velocity
	x = x + dx*dt; %Update position
	e = e + (xhat-x); %Cumulated tracking error
end
disp(['Cumulated tracking error for ideal underdamped system: ' num2str(e)]);

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; hold on;
plot([1:nbData]*dt, Data(1,:), 'k-');
plot([1:nbData]*dt, Data2(1,:), 'k:');
legend('critically damped', 'ideal underdamped');
plot([1,nbData]*dt, [xhat,xhat], 'r-');
xlabel('t'); ylabel('x');

%print('-dpng','graphs/demo_testDampingRatio01.png');
%pause;
%close all;
	
	
	
	