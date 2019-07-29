function h=plotGMM3D(Mu, Sigma, col1, alpha, dispOpt)
%Sylvain Calinon, 2015

if nargin<4
	alpha=1;
end
if nargin<5
	dispOpt=1;
end

nbData = size(Mu,2);
nbPoints = 20; %nb of points to form a circular path
nbRings = 10; %Number of circular paths following the principal direction

pts0 = [cos(linspace(0,2*pi,nbPoints)); sin(linspace(0,2*pi,nbPoints))];

h=[];
for n=1:nbData
  [V0,D0] = eigs(Sigma(:,:,n));
  U0 = real(V0*D0^.5);

  ringpts0 = [cos(linspace(0,pi,nbRings+1)); sin(linspace(0,pi,nbRings+1))];
  ringpts = zeros(3,nbRings); 
  ringpts([2,3],:) = ringpts0(:,1:nbRings);
  U = zeros(3); 
  U(:,[2,3]) = U0(:,[2,3]); 
  ringTmp = U*ringpts; 
 
  %Compute touching circular paths
  for j=1:nbRings
    U = zeros(3); 
    U(:,1) = U0(:,1); 
    U(:,2) = ringTmp(:,j);
    pts = zeros(3,nbPoints); 
    pts([1,2],:) = pts0;
    xring(:,:,j) = U*pts + repmat(Mu(:,n),1,nbPoints);
  end

  %Plot filled ellispoid
  xringfull = xring;
  xringfull(:,:,end+1) = xringfull(:,end:-1:1,1); %Close the ellipsoid
  for j=1:size(xringfull,3)-1
    for i=1:size(xringfull,2)-1
      xTmp = [xringfull(:,i,j) xringfull(:,i+1,j) xringfull(:,i+1,j+1) xringfull(:,i,j+1) xringfull(:,i,j)];
			if dispOpt==1
				%Version 1 (for GMM plots)
				h = [h patch(xTmp(1,:),xTmp(2,:),xTmp(3,:), min(col1+0.1,1),'edgecolor',col1,'linewidth',1,'facealpha',alpha,'edgealpha',alpha)]; %,'facealpha',0.5
			else
				%Version 2 (for GMR plots)
				patch(xTmp(1,:),xTmp(2,:),xTmp(3,:), min(col1+0.35,1),'linestyle','none','facealpha',alpha);
			end
    end
  end
end
