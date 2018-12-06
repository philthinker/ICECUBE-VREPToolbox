function [varargout] = rcp(action,varargin)
% A rhythmic movement primitive (RCP) as suggested in
% Ijspeert A, Nakanishi J, Schaal S (2003) Learning attractor landscapes for 
% learning motor primitives. In: Becker S, Thrun S, Obermayer K (eds) Advances 
% in Neural Information Processing Systems 15. MIT Press, Cambridge, MA.
% http://www-clmc.usc.edu/publications/I/ijspeert-NIPS2002.pdf
% This version simplfies the primitive and only uses a one dimensional
% function fitting.
% 
% Copyright June, 2006 by
%           Stefan Schaal and Auke Ijspeert
%           
% ---------------  Different Actions of the program ------------------------
%
% Initialize a rcp model:
% FORMAT rcp('Init',ID,n_rfs,name)
% ID              : desired ID of model
% n_rfs           : number of local linear models
% name            : a name for the model
%
% alternatively, the function is called as
%
% FORMAT rcp('Init',ID,d,)
% d               : a complete data structure of a rcp model
%
% returns nothing
% -------------------------------------------------------------------------
%
% Set the amplitude:
% FORMAT rcp('Set_Amplitude',ID,A)
% ID              : ID of model
% A               : the new amplitude
%
% returns nothing
% -------------------------------------------------------------------------
%
% Set the baseline:
% FORMAT rcp('Set_Baseline',ID,ym)
% ID              : ID of model
% ym              : the new baseline for the oscillation
%
% returns nothing
% -------------------------------------------------------------------------
%
% Run the rcp:
% FORMAT [[y,yd,ydd]=rcp('Run',ID,tau,dt)
% ID              : ID of model
% tau             : global time constant to scale speed of system
% dt              : integration time step
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the rcp
% -------------------------------------------------------------------------
%
% Change values of a rcp:
% FORMAT rcp('Change',ID,pname,value)
% ID              : ID of model
% pname           : parameter name
% value           : value to be assigned to parameter
%
% returns nothing
% -------------------------------------------------------------------------
%
% Run the rcps:
% FORMAT rcp('Run',ID,tau,dt,t,td)
% ID              : ID of model
% tau             : time constant to scale speed, roughly equivalent to
%                   period of movement
% dt              : integration time step
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the rcp
% -------------------------------------------------------------------------
%
% Run the rcp and update the weights:
% FORMAT rcp('Run_Fit',ID,tau,dt,t,td,tdd)
% ID              : ID of model
% tau             : time constant to scale speed
% dt              : integration time step
% t               : target for y
% td              : target for yd
% tdd             : target for ydd
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the rcp
% -------------------------------------------------------------------------
%
% Fit the rcp to a complete trajectory in batch mode:
% FORMAT rcp('Batch_Fit',ID,tau,dt,T,Td)
% ID              : ID of model
% tau             : time constant to scale speed, tau is roughly movement
%                   time until convergence the goal
% dt              : somple time step in given trajectory
% T               : target trajectory for y
% Td              : target trajectory for yd (optional, will be generated
%                   as dT/dt otherwise
% Tdd             : target trajectory for ydd (optional, will be generated
%                   as dTd/dt otherwise
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the rcp
% -------------------------------------------------------------------------
%
% Return the data structure of a rcp model
% FORMAT [d] = rcps('Structure',ID)
% ID              : desired ID of model
%
% returns the complete data structure of a rcp model, e.g., for saving or
% inspecting it
% -------------------------------------------------------------------------
%
% Reset the states of a rcp model to zero
% FORMAT [d] = rcps('Reset_State',ID)
% ID              : desired ID of model
%
% returns nothing
% -------------------------------------------------------------------------
%
% Clear the data structure of a LWPR model
% FORMAT rcps('Clear',ID)
% ID              : ID of model
%
% returns nothing
% -------------------------------------------------------------------------
%
% Initializes the rcp with a sine trajectory
% FORMAT rcps('Sine',ID)
% ID              : ID of model
%
% returns nothing
% -------------------------------------------------------------------------

% the global structure to store all rcps
global rcps;

% at least two arguments are needed
if nargin < 2,
  error('Incorrect call to rcp');
end

switch lower(action),
% .........................................................................
  case 'init'
    if (nargin == 3) 
      ID        = varargin{1};
      rcps(ID)  = varargin{2};
    else 
      % this initialization is good for period 1 second movement for tau=1
      ID               = varargin{1};
      n_rfs            = varargin{2};
      rcps(ID).name    = varargin{3};
      % the time constants for chosen for critical damping
      rcps(ID).alpha_z = 25;
      rcps(ID).beta_z  = rcps(ID).alpha_z/4;
      rcps(ID).omega   = 2*pi;
      % initialize the state variables
      rcps(ID).z       = 0;
      rcps(ID).y       = 0; 
      rcps(ID).p       = 0;
      rcps(ID).zd      = 0;
      rcps(ID).yd      = 0; 
      rcps(ID).pd      = 0;
      rcps(ID).ydd     = 0; 
      % baseline and amplitude of oscillation
      rcps(ID).ym      = 0;
      rcps(ID).A       = 0;

      % the local models, spaced on grid in phase space
      rcps(ID).c       = 2*pi*(1/n_rfs:1/n_rfs:1)';
      rcps(ID).psi     = zeros(n_rfs,1);
      rcps(ID).w       = zeros(n_rfs,1);
      rcps(ID).sp2     = zeros(n_rfs,1);
      rcps(ID).sptd    = zeros(n_rfs,1);
      rcps(ID).D       = ones(n_rfs,1)*.1*n_rfs^2;
      rcps(ID).lambda  = 1;
    end
    
% .........................................................................
  case 'reset_state'
    ID               = varargin{1};
    % initialize the state variables
    rcps(ID).z       = 0;
    rcps(ID).y       = 0; 
    rcps(ID).p       = 0;
    rcps(ID).zd      = 0;
    rcps(ID).yd      = 0; 
    rcps(ID).pd      = 0;
    rcps(ID).ydd     = 0; 
    % the baseline
    rcps(ID).ym      = 0;
    
% .........................................................................
  case 'set_baseline'
    ID               = varargin{1};
    rcps(ID).ym      = varargin{2};
    
% .........................................................................
  case 'set_amplitude'
    ID               = varargin{1};
    rcps(ID).A       = varargin{2};
        
% .........................................................................
  case 'run'
    ID               = varargin{1};
    tau              = varargin{2}; 
    dt               = varargin{3};
    if nargin > 4,
      random_value  = varargin{4};
    else
      random_value  = 0;
    end
    
    % the weighted sum of the locally weighted regression models
    rcps(ID).psi = exp(rcps(ID).D.*(cos(rcps(ID).p-rcps(ID).c)-1));
    amp          = rcps(ID).A;
    f            = sum(rcps(ID).w.*rcps(ID).psi)/sum(rcps(ID).psi+1.e-10) * amp;
    
    rcps(ID).pd = rcps(ID).omega*tau;
    
    rcps(ID).zd = (rcps(ID).alpha_z*(rcps(ID).beta_z*(rcps(ID).ym-rcps(ID).y)-rcps(ID).z)+f)*tau ...
	                + random_value;
    rcps(ID).yd = rcps(ID).z*tau;
    rcps(ID).ydd= rcps(ID).zd*tau;
        
    rcps(ID).p  = rcps(ID).pd*dt+rcps(ID).p;
    
    rcps(ID).z  = rcps(ID).zd*dt+rcps(ID).z;
    rcps(ID).y  = rcps(ID).yd*dt+rcps(ID).y;
        
    varargout(1) = {rcps(ID).y};
    varargout(2) = {rcps(ID).yd};
    varargout(3) = {rcps(ID).ydd};
    %varargout(4) = {rcps(ID).psi*rcps(ID).v/sum(rcps(ID).psi)};
    varargout(4) = {amp*rcps(ID).psi};
    
% .........................................................................
  case 'change'
    ID      = varargin{1};
    command = sprintf('rcps(%d).%s = varargin{3};',ID,varargin{2});
    eval(command);
    
% .........................................................................
  case 'run_fit'
    ID               = varargin{1};
    tau              = varargin{2}; 
    dt               = varargin{3};
    t                = varargin{4};
    td               = varargin{5};
    tdd              = varargin{6};
        
    % the regression target
    amp              = rcps(ID).A;
    ft               = (tdd/tau^2-rcps(ID).alpha_z*(rcps(ID).beta_z*(rcps(ID).ym-t)-td/tau))/amp;
    
    % the weighted sum of the locally weighted regression models
    rcps(ID).psi = exp(rcps(ID).D.*(cos(rcps(ID).p-rcps(ID).c)-1));
    
    % update the regression
    rcps(ID).sp2  = rcps(ID).sp2*rcps(ID).lambda + rcps(ID).psi;
    rcps(ID).sptd = rcps(ID).sptd*rcps(ID).lambda + rcps(ID).psi*ft;
    rcps(ID).w    = rcps(ID).sptd./(rcps(ID).sp2+1.e-10);
    
    % compute nonlinearity
    f     = sum(rcps(ID).w.*rcps(ID).psi)/sum(rcps(ID).psi+1.e-10) * amp;
    
    % integrate
    rcps(ID).pd = rcps(ID).omega*tau;
    
    % note that yd = td = z*tau   ==> z=td/tau; the first equation means
    % simply rcps(ID).zd = tdd
    rcps(ID).zd = (rcps(ID).alpha_z*(rcps(ID).beta_z*(rcps(ID).ym-rcps(ID).y)-rcps(ID).z)+f)*tau;
    rcps(ID).yd = rcps(ID).z*tau;
    rcps(ID).ydd= rcps(ID).zd*tau;
    
    rcps(ID).p  = rcps(ID).pd*dt+rcps(ID).p;
    
    rcps(ID).z  = rcps(ID).zd*dt+rcps(ID).z;
    rcps(ID).y  = rcps(ID).yd*dt+rcps(ID).y;
    
    varargout(1) = {rcps(ID).y};
    varargout(2) = {rcps(ID).yd};
    varargout(3) = {rcps(ID).ydd};
    
% .........................................................................
  case 'batch_fit'
    
    ID               = varargin{1};
    tau              = varargin{2};
    dt               = varargin{3};
    T                = varargin{4};
    if (nargin > 4) 
      Td               = varargin{5};
    else
      Td               = diffnc(T,dt);
    end
    if (nargin > 5) 
      Tdd              = varargin{6};
    else
      Tdd              = diffnc(Td,dt);
    end
    
    % the amplitude
    A    = rcps(ID).A;
    
    % the baseline
    ym   = rcps(ID).ym;
     
    % compute the hidden states
    P = zeros(size(T));
    p = 0;
   
    for i=1:length(T),
      P(i) = p;
      pd   = rcps(ID).omega*tau;
      p    = pd*dt+p;
    end
    
    % the regression target
    amp = A;
    Ft  = (Tdd/tau^2-rcps(ID).alpha_z*(rcps(ID).beta_z*(ym-T)-Td/tau)) / amp;
    
    % compute the weights for each local model along the trajectory
    PSI = exp((cos(P*ones(1,length(rcps(ID).c))-ones(length(T),1)*rcps(ID).c')-1).*(ones(length(T),1)*rcps(ID).D'));
    
    
    % compute the regression
    rcps(ID).sp2  = sum((ones(length(P),length(rcps(ID).c))).*PSI,1)';
    rcps(ID).sptd = sum((Ft*ones(1,length(rcps(ID).c))).*PSI,1)';
    rcps(ID).w    = rcps(ID).sptd./(rcps(ID).sp2+1.e-10);
    
    
    % compute the prediction
    F     = sum((ones(length(P),1)*rcps(ID).w').*PSI,2)./sum(PSI,2) * amp;
    z     = 0;
    zd    = 0;
    y     = T(1);
    Y     = zeros(size(T));
    Yd    = zeros(size(T));
    Ydd   = zeros(size(T));
    
    for i=1:length(T),
      
      Ydd(i) = zd*tau;
      Yd(i)  = z;
      Y(i)   = y;
      
      zd   = (rcps(ID).alpha_z*(rcps(ID).beta_z*(ym-y)-z)+F(i))*tau;
      yd   = z;
      
      z    = zd*dt+z;
      y    = yd*dt+y;
            
    end
        
    varargout(1) = {Y};
    varargout(2) = {Yd};
    varargout(3) = {Ydd};
    
% .........................................................................
  case 'structure'
    ID     = varargin{1};
    varargout(1) = {rcps(ID)};	
    
% .........................................................................
  case 'clear'
    ID     = varargin{1};
    rcps(ID) = [];
    
% .........................................................................
  case 'sine'
    ID     = varargin{1};
    
    % generate the minimum jerk trajectory as target to learn from
    t=0;
    td=0;
    tdd=0;
    A = 1;
    ym = 0;
    
    rcp('reset_state',ID);
    rcp('set_amplitude',ID,A);
    rcp('set_baseline',ID,ym);
    tau = 1;
    dt = 0.001;
    T=zeros(2*tau/dt,3);

    for i=0:2*tau/dt,
      t = i*dt;
      T(i+1,:)   = [A*sin(2*pi/tau*t) A*(2*pi/tau)*cos(2*pi/tau*t) -A*(2*pi/tau)^2*sin(2*pi/tau*t)];
    end;

    % batch fitting
    i = round(2*tau/dt); % only fit the part of the trajectory with the signal
    [Yp,Ypd,Ypdd]=rcp('batch_fit',ID,tau,dt,T(1:i,1),T(1:i,2),T(1:i,3));
    
% .........................................................................
  otherwise
    error('unknown action');
    
end