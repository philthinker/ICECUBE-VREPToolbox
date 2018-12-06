function [varargout] = dcp(action,varargin)
% A discrete movement primitive (DCP) inspired by
% Ijspeert A, Nakanishi J, Schaal S (2003) Learning attractor landscapes for 
% learning motor primitives. In: Becker S, Thrun S, Obermayer K (eds) Advances 
% in Neural Information Processing Systems 15. MIT Press, Cambridge, MA.
% http://www-clmc.usc.edu/publications/I/ijspeert-NIPS2002.pdf. This
% version adds several new features, including that the primitive is
% formulated as acceleration, and that the canonical system is normalized.
% Additinally, a new scale parameter for the nonlinear function allows a larger 
% spectrum of modeling options with the primitives
% 
% Copyright July, 2007 by
%           Stefan Schaal, Auke Ijspeert, and Heiko Hoffmann
%           
% ---------------  Different Actions of the program ------------------------
%
% Initialize a DCP model:
% FORMAT dcp('Init',ID,n_rfs,name,flag)
% ID              : desired ID of model
% n_rfs           : number of local linear models
% name            : a name for the model
% flag            : flag=1 use 2nd order canonical system, flag=0 use 1st order
%
% alternatively, the function is called as
%
% FORMAT dcp('Init',ID,d,)
% d               : a complete data structure of a dcp model
%
% returns nothing
% -------------------------------------------------------------------------
%
% Set the goal state:
% FORMAT dcp('Set_Goal',ID,g,flag)
% ID              : ID of model
% g               : the new goal
% flag            : flag=1: update x0 with current state, flag=0: don't update x0
%
% returns nothing
% -------------------------------------------------------------------------
%
% Set the scale factor of the movement:
% FORMAT dcp('Set_Scale',ID,s,flag)
% ID              : ID of model
% s               : the new scale
%
% returns nothing
% -------------------------------------------------------------------------
%
% Run the dcps:
% FORMAT [y,yd,ydd]=dcp('Run',ID,tau,dt,ct,cc)
% ID              : ID of model
% tau             : global time constant to scale speed of system
% dt              : integration time step
% ct              : coupling term for transformation system (optional)
% cc              : coupling term for canonical system (optional)
% ct_tau          : coupling term for transformation system's time constant (optional)
% cc_tau          : coupling term for canonical system's time constant (optional)
% cw              : additive coupling term for parameters (optional)
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the dcp
% -------------------------------------------------------------------------
%
% Change values of a dcp:
% FORMAT dcp('Change',ID,pname,value)
% ID              : ID of model
% pname           : parameter name
% value           : value to be assigned to parameter
%
% returns nothing
% -------------------------------------------------------------------------
%
% Run the dcps:
% FORMAT dcp('Run',ID,tau,dt,t,td)
% ID              : ID of model
% tau             : time constant to scale speed, tau is roughly movement
%                   time until convergence
% dt              : integration time step
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the dcp
% -------------------------------------------------------------------------
%
% Run the dcp and update the weights:
% FORMAT dcp('Run_Fit',ID,tau,dt,t,td,tdd)
% ID              : ID of model
% tau             : time constant to scale speed, tau is roughly movement
%                   time until convergence
% dt              : integration time step
% t               : target for y
% td              : target for yd
% tdd             : target for ydd
%
% returns y,yd,ydd, i.e., current pos,vel,acc, of the dcp
% -------------------------------------------------------------------------
%
% Fit the dcp to a complete trajectory in batch mode:
% FORMAT dcp('Batch_Fit',ID,tau,dt,T,Td,Tdd)
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
% returns y,yd,ydd, i.e., current pos,vel,acc, of the dcp
% -------------------------------------------------------------------------
%
% Return the data structure of a dcp model
% FORMAT [d] = dcp('Structure',ID)
% ID              : desired ID of model
%
% returns the complete data structure of a dcp model, e.g., for saving or
% inspecting it
% -------------------------------------------------------------------------
%
% Reset the states of a dcp model to zero (or a given state)
% FORMAT [d] = dcp('Reset_State',ID)
% ID              : desired ID of model
% y               : the state to which the primitive is set (optional)
%
% returns nothing
% -------------------------------------------------------------------------
%
% Clear the data structure of a LWPR model
% FORMAT dcp('Clear',ID)
% ID              : ID of model
%
% returns nothing
% -------------------------------------------------------------------------
%
% Initializes the dcp with a minimum jerk trajectory
% FORMAT dcp('MinJerk',ID)
% ID              : ID of model
%
% returns nothing
% -------------------------------------------------------------------------

% the global structure to store all dcps
global dcps;
global min_y;
global max_y;

% at least two arguments are needed
if nargin < 2,
  error('Incorrect call to dcp');
end

switch lower(action),
% .........................................................................
  case 'init'
    if (nargin == 3) 
      ID        = varargin{1};
      dcps(ID)  = varargin{2};
    else 
      % this initialization is good for 0.5 seconds movement for tau=0.5
      ID               = varargin{1};
      n_rfs            = varargin{2};
      dcps(ID).name    = varargin{3};
      dcps(ID).c_order = 0;
      if (nargin == 5)
        dcps(ID).c_order = varargin{4};
      end
      % the time constants for chosen for critical damping
      dcps(ID).alpha_z = 25;
      dcps(ID).beta_z  = dcps(ID).alpha_z/4;
      dcps(ID).alpha_g = dcps(ID).alpha_z/2;
      dcps(ID).alpha_x = dcps(ID).alpha_z/3;
      dcps(ID).alpha_v = dcps(ID).alpha_z;
      dcps(ID).beta_v  = dcps(ID).beta_z;
      % initialize the state variables
      dcps(ID).z       = 0;
      dcps(ID).y       = 0; 
      dcps(ID).x       = 0;
      dcps(ID).v       = 0;
      dcps(ID).zd      = 0;
      dcps(ID).yd      = 0; 
      dcps(ID).xd      = 0;
      dcps(ID).vd      = 0;
      dcps(ID).ydd     = 0; 
      % the current goal state
      dcps(ID).g       = 0;
      dcps(ID).gd      = 0;
      dcps(ID).G       = 0;
      % the current start state of the primitive
      dcps(ID).y0      = 0;
      % the orginal amplitude (max(y)-min(y)) when the primitive was fit
      dcps(ID).A       = 0;
      % the original goal amplitude (G-y0) when the primitive was fit
      dcps(ID).dG      = 0;
      % the scale factor for the nonlinear function
      dcps(ID).s       = 1;
      
      t = (0:1/(n_rfs-1):1)'*0.5;
      if (dcps(ID).c_order == 1)
        % the local models, spaced on a grid in time by applying the
        % anaytical solutions x(t) = 1-(1+alpha/2*t)*exp(-alpha/2*t)
        dcps(ID).c       = (1+dcps(ID).alpha_z/2*t).*exp(-dcps(ID).alpha_z/2*t);
        % we also store the phase velocity at the centers which is used by some
        % applications: xd(t) = (-alpha/2)*x(t) + alpha/2*exp(-alpha/2*t)
        dcps(ID).cd      = dcps(ID).c*(-dcps(ID).alpha_z/2) + dcps(ID).alpha_z/2*exp(-dcps(ID).alpha_z/2*t);
      else
        % the local models, spaced on a grid in time by applying the
        % anaytical solutions x(t) = exp(-alpha*t)
        dcps(ID).c       = exp(-dcps(ID).alpha_x*t);
        % we also store the phase velocity at the centers which is used by some
        % applications: xd(t) = x(t)*(-dcps(ID).alpha_x);
        dcps(ID).cd      = dcps(ID).c*(-dcps(ID).alpha_x);
      end
      
      dcps(ID).psi     = zeros(n_rfs,1);
      dcps(ID).w       = zeros(n_rfs,1);
      dcps(ID).sx2     = zeros(n_rfs,1);
      dcps(ID).sxtd    = zeros(n_rfs,1);
      dcps(ID).D       = (diff(dcps(ID).c)*0.55).^2;
      dcps(ID).D       = 1./[dcps(ID).D;dcps(ID).D(end)];
      dcps(ID).lambda  = 1;
      
    end
    
% .........................................................................
  case 'reset_state'
    ID               = varargin{1};
    if nargin > 2,
      y = varargin{2};
    else
      y = 0;
    end
    % initialize the state variables
    dcps(ID).z       = 0;
    dcps(ID).y       = y; 
    dcps(ID).x       = 0;
    dcps(ID).v       = 0;
    dcps(ID).zd      = 0;
    dcps(ID).yd      = 0; 
    dcps(ID).xd      = 0;
    dcps(ID).vd      = 0;
    dcps(ID).ydd     = 0; 
    % the goal state
    dcps(ID).G       = y;
    dcps(ID).g       = y;
    dcps(ID).gd      = 0;
    dcps(ID).y0      = y;
    dcps(ID).s       = 1;
    
% .........................................................................
  case 'set_goal'
    ID               = varargin{1};
    dcps(ID).G       = varargin{2};
    if (dcps(ID).c_order == 0)
      dcps(ID).g     = dcps(ID).G;
    end
    flag             = varargin{3};
    if (flag),
      dcps(ID).x     = 1;
      dcps(ID).y0    = dcps(ID).y;
    end
    if (dcps(ID).A ~= 0)  % check whether dcp has been fit
      if (dcps(ID).A/(abs(dcps(ID).dG)+1.e-10) > 2.0) 
        % amplitude-based scaling needs to be set explicity
      else
        % dG based scaling cab work automatically
        dcps(ID).s       = (dcps(ID).G-dcps(ID).y0)/dcps(ID).dG;
      end
    end
    
% .........................................................................
  case 'set_scale'
    ID               = varargin{1};
    dcps(ID).s       = varargin{2};
        
% .........................................................................
  case 'run'
    ID               = varargin{1};
    tau              = 0.5/varargin{2}; % tau is relative to 0.5 seconds nominal movement time
    dt               = varargin{3};
    
    if nargin > 4,
      ct  = varargin{4};
    else
      ct  = 0;
    end

    if nargin > 5,
      cc  = varargin{5};
    else
      cc  = 0;
    end
    
    if nargin > 6,
      ct_tau  = varargin{6};
    else
      ct_tau  = 1;
    end
    
    if nargin > 7,
      cc_tau  = varargin{7};
    else
      cc_tau  = 1;
    end
    
    if nargin > 8,
      cw  = varargin{8};
    else
      cw  = 0;
    end
    
    % the weighted sum of the locally weighted regression models
    dcps(ID).psi = exp(-0.5*((dcps(ID).x-dcps(ID).c).^2).*dcps(ID).D);
    amp          = dcps(ID).s;
    if (dcps(ID).c_order == 1)
      in = dcps(ID).v;
    else
      in = dcps(ID).x;
    end
    f            = sum(in*(dcps(ID).w+cw).*dcps(ID).psi)/sum(dcps(ID).psi+1.e-10) * amp;
    
    if (dcps(ID).c_order == 1),
      dcps(ID).vd = (dcps(ID).alpha_v*(dcps(ID).beta_v*(0-dcps(ID).x)-dcps(ID).v)+cc)*tau*cc_tau;
      dcps(ID).xd = dcps(ID).v*tau*cc_tau;
    else 
      dcps(ID).vd = 0;
      dcps(ID).xd = (dcps(ID).alpha_x*(0-dcps(ID).x)+cc)*tau*cc_tau;
    end
    
    dcps(ID).zd = (dcps(ID).alpha_z*(dcps(ID).beta_z*(dcps(ID).g-dcps(ID).y)-dcps(ID).z)+f+ct)*tau*ct_tau;
    dcps(ID).yd = dcps(ID).z*tau*ct_tau;
    dcps(ID).ydd= dcps(ID).zd*tau*ct_tau;
    
    dcps(ID).gd = dcps(ID).alpha_g*(dcps(ID).G-dcps(ID).g);
    
    dcps(ID).x  = dcps(ID).xd*dt+dcps(ID).x;
    dcps(ID).v  = dcps(ID).vd*dt+dcps(ID).v;
    
    
    dcps(ID).z  = dcps(ID).zd*dt+dcps(ID).z;
    dcps(ID).y  = dcps(ID).yd*dt+dcps(ID).y;
    
    dcps(ID).g  = dcps(ID).gd*dt+dcps(ID).g;

        
    varargout(1) = {dcps(ID).y};
    varargout(2) = {dcps(ID).yd};
    varargout(3) = {dcps(ID).ydd};
    varargout(4) = {dcps(ID).psi*in/sum(dcps(ID).psi+1.e-10) * amp};
    
% .........................................................................
  case 'change'
    ID      = varargin{1};
    command = sprintf('dcps(%d).%s = varargin{3};',ID,varargin{2});
    eval(command);
    
% .........................................................................
  case 'run_fit'
    ID               = varargin{1};
    tau              = 0.5/varargin{2}; % tau is relative to 0.5 seconds nominal movement time
    dt               = varargin{3};
    t                = varargin{4};
    td               = varargin{5};
    tdd              = varargin{6};
    
    % check whether this is the first time the primitive is fit, and record the
    % amplitude and dG information
    if (dcps(ID).A == 0)
      dcps(ID).dG = dcps(ID).G - dcps(ID).y0;
      if (dcps(ID).x == 1),
        min_y = +1.e10;
        max_y = -1.e10;
        dcps(ID).s = 1;
      end
    end
        
    % the regression target
    amp              = dcps(ID).s;
    ft               = (tdd/tau^2-dcps(ID).alpha_z*(dcps(ID).beta_z*(dcps(ID).g-t)-td/tau))/amp;
    
    % the weighted sum of the locally weighted regression models
    dcps(ID).psi = exp(-0.5*((dcps(ID).x-dcps(ID).c).^2).*dcps(ID).D);
    
    % update the regression
    if (dcps(ID).c_order == 1),
      dcps(ID).sx2  = dcps(ID).sx2*dcps(ID).lambda + dcps(ID).psi*dcps(ID).v^2;
      dcps(ID).sxtd = dcps(ID).sxtd*dcps(ID).lambda + dcps(ID).psi*dcps(ID).v*ft;
      dcps(ID).w    = dcps(ID).sxtd./(dcps(ID).sx2+1.e-10);
    else
      dcps(ID).sx2  = dcps(ID).sx2*dcps(ID).lambda + dcps(ID).psi*dcps(ID).x^2;
      dcps(ID).sxtd = dcps(ID).sxtd*dcps(ID).lambda + dcps(ID).psi*dcps(ID).x*ft;
      dcps(ID).w    = dcps(ID).sxtd./(dcps(ID).sx2+1.e-10);
    end
    
    % compute nonlinearity
    if (dcps(ID).c_order == 1)
      in = dcps(ID).v;
    else
      in = dcps(ID).x;
    end
    f     = sum(in*dcps(ID).w.*dcps(ID).psi)/sum(dcps(ID).psi+1.e-10) * amp;
    
    % integrate
    if (dcps(ID).c_order == 1),
      dcps(ID).vd = (dcps(ID).alpha_v*(dcps(ID).beta_v*(0-dcps(ID).x)-dcps(ID).v))*tau;
      dcps(ID).xd = dcps(ID).v*tau;
    else 
      dcps(ID).vd = 0;
      dcps(ID).xd = dcps(ID).alpha_x*(0-dcps(ID).x)*tau;
    end
    
    % note that yd = td = z*tau   ==> z=td/tau; the first equation means
    % simply dcps(ID).zd = tdd
    dcps(ID).zd = (dcps(ID).alpha_z*(dcps(ID).beta_z*(dcps(ID).g-dcps(ID).y)-dcps(ID).z)+f)*tau;
    dcps(ID).yd = dcps(ID).z*tau;
    dcps(ID).ydd= dcps(ID).zd*tau;
    
    dcps(ID).gd = dcps(ID).alpha_g*(dcps(ID).G-dcps(ID).g);
    
    dcps(ID).x  = dcps(ID).xd*dt+dcps(ID).x;
    dcps(ID).v  = dcps(ID).vd*dt+dcps(ID).v;
    
    dcps(ID).z  = dcps(ID).zd*dt+dcps(ID).z;
    dcps(ID).y  = dcps(ID).yd*dt+dcps(ID).y;
    
    dcps(ID).g  = dcps(ID).gd*dt+dcps(ID).g;

    varargout(1) = {dcps(ID).y};
    varargout(2) = {dcps(ID).yd};
    varargout(3) = {dcps(ID).ydd};
    
    if (dcps(ID).A == 0)
      max_y = max(max_y,dcps(ID).y);
      min_y = min(min_y,dcps(ID).y);
      if (dcps(ID).x < 0.0001)
        dcps(ID).A = max_y - min_y;
      end
    end
    
% .........................................................................
  case 'batch_fit'
    
    ID               = varargin{1};
    tau              = 0.5/varargin{2}; % tau is relative to 0.5 seconds nominal movement time
    dt               = varargin{3};
    T                = varargin{4};
    if (nargin > 5) 
      Td               = varargin{5};
    else
      Td               = diffnc(T,dt);
    end
    if (nargin > 6) 
      Tdd              = varargin{6};
    else
      Tdd              = diffnc(Td,dt);
    end
    
    % the start state is the first state in the trajectory
    y0 = T(1);
    g  = y0;
    
    % the goal is the last state in the trajectory
    goal = T(end);
    if (dcps(ID).c_order == 0)
      g = goal;
    end
    
    % the amplitude is the max(T)-min(T)
    A    = max(T)-min(T);
     
    % compute the hidden states
    X = zeros(size(T));
    V = zeros(size(T));
    G = zeros(size(T));
    x = 1;
    v = 0;
   
    for i=1:length(T),
      
      X(i) = x;
      V(i) = v;
      G(i) = g;
      
      if (dcps(ID).c_order == 1)
        vd   = dcps(ID).alpha_v*(dcps(ID).beta_v*(0-x)-v)*tau;
        xd   = v*tau;
      else
        vd   = 0;
        xd   = dcps(ID).alpha_x*(0-x)*tau;
      end
      gd   = (goal - g) * dcps(ID).alpha_g;
      
      x    = xd*dt+x;
      v    = vd*dt+v;
      g    = gd*dt+g;
      
    end
    
    % the regression target
    dcps(ID).dG = goal - y0;
    dcps(ID).A  = max(T)-min(T);
    dcps(ID).s  = 1;  % for fitting a new primitive, the scale factor is always equal to one

    amp = dcps(ID).s;
    Ft  = (Tdd/tau^2-dcps(ID).alpha_z*(dcps(ID).beta_z*(G-T)-Td/tau)) / amp;
    
    % compute the weights for each local model along the trajectory
    PSI = exp(-0.5*((X*ones(1,length(dcps(ID).c))-ones(length(T),1)*dcps(ID).c').^2).*(ones(length(T),1)*dcps(ID).D'));

    % compute the regression
    if (dcps(ID).c_order == 1)
      dcps(ID).sx2  = sum(((V.^2)*ones(1,length(dcps(ID).c))).*PSI,1)';
      dcps(ID).sxtd = sum(((V.*Ft)*ones(1,length(dcps(ID).c))).*PSI,1)';
      dcps(ID).w    = dcps(ID).sxtd./(dcps(ID).sx2+1.e-10);
    else
      dcps(ID).sx2  = sum(((X.^2)*ones(1,length(dcps(ID).c))).*PSI,1)';
      dcps(ID).sxtd = sum(((X.*Ft)*ones(1,length(dcps(ID).c))).*PSI,1)';
      dcps(ID).w    = dcps(ID).sxtd./(dcps(ID).sx2+1.e-10);
    end
    
    % compute the prediction
    if (dcps(ID).c_order == 1)
      F     = sum((V*dcps(ID).w').*PSI,2)./sum(PSI,2) * amp;      
    else
      F     = sum((X*dcps(ID).w').*PSI,2)./sum(PSI,2) * amp;
    end
    z     = 0;
    zd    = 0;
    y     = y0;
    Y     = zeros(size(T));
    Yd    = zeros(size(T));
    Ydd   = zeros(size(T));
    
    for i=1:length(T),
      
      Ydd(i) = zd*tau;
      Yd(i)  = z;
      Y(i)   = y;
      
      zd   = (dcps(ID).alpha_z*(dcps(ID).beta_z*(G(i)-y)-z)+F(i))*tau;
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
    varargout(1) = {dcps(ID)};	
    
% .........................................................................
  case 'clear'
    ID     = varargin{1};
    if exist('dcps')
      if length(dcps) >= ID,
        dcps(ID) = [];
      end
    end
    
% .........................................................................
  case 'minjerk'
    ID     = varargin{1};
    
    % generate the minimum jerk trajectory as target to learn from
    t=0;
    td=0;
    tdd=0;
    goal = 1;
    
    dcp('reset_state',ID);
    dcp('set_goal',ID,goal,1);
    tau = 0.5;
    dt = 0.001;
    T=zeros(2*tau/dt,3);

    for i=0:2*tau/dt,
      [t,td,tdd]=min_jerk_step(t,td,tdd,goal,tau-i*dt,dt);
      T(i+1,:)   = [t td tdd];
    end;

    % batch fitting
    i = round(2*tau/dt); % only fit the part of the trajectory with the signal
    [Yp,Ypd,Ypdd]=dcp('batch_fit',ID,tau,dt,T(1:i,1),T(1:i,2),T(1:i,3));
    
% .........................................................................
  otherwise
    error('unknown action');
    
end