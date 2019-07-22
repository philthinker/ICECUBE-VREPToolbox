function [I,J] = SAlign(X,Y,varargin)
%SAMPLEALIGN aligns two data sets containing sequential observations.
%
%  [I,J] = SAMPLEALIGN(X,Y) aligns the observations in two matrices of
%  data, X and Y, by introducing gaps. X and Y are matrices of data where
%  rows correspond to observations or samples, and columns correspond to
%  features or dimensions. X and Y can have different number of rows, but
%  must have the same number of columns. The first column is the reference
%  dimension and must contain unique values in ascending order. The
%  reference dimension could contain sample indices of the observations or
%  a measurable value, such as time. The SAMPLEALIGN function uses a
%  dynamic programming algorithm to minimize the sum of positive scores
%  resulting from pairs of observations that are potential matches and the
%  penalties resulting from the insertion of gaps. Return values I and J
%  are column vectors containing indices that indicate the matches for each
%  row (observation) in X and Y respectively. When you do not specify
%  return values, SAMPLEALIGN does not run the dynamic programming
%  algorithm, however you can explore the constrained space, the dynamic
%  programming network and the observations to align by using the
%  'SHOWCONSTRAINTS', 'SHOWNETWORK', and 'SHOWALIGNMENT' input arguments.
%
%  SAMPLEALIGN(...,'BAND',B) specifies a maximum allowable distance between
%  observations along the reference dimension, thus limiting the number
%  potential matches between observations in two data sets. Let S be the
%  value in the reference dimension for any given observation (row) in one
%  data set, then that observation is matched only with observations in the
%  other data set whose values in the reference dimension fall within S +/-
%  B. Only these potential matches are passed to the algorithm for further
%  scoring. B can be a scalar or a function specified using @(z), where z
%  is the mid-point between a given observation in one data set and a given
%  observation in the other data set. Default B is Inf.
%
%  The 'BAND' constraint reduces the time and memory complexity of the
%  algorithm from O(MN) to O(sqrt(MN)*K), where M and N are the number of
%  observations in X and Y respectively, and K is a small constant such
%  that K<<M and K<<N. Adjust B to the maximum expected shift between
%  X(:,1) and Y(:,1).
%
%  SAMPLEALIGN(...,'WIDTH',[U,V]) limits the number of potential matches
%  between observations in two data sets; that is, each observation in X is
%  scored to the closest U observations in Y, and each observation in Y is
%  scored to the closest V observations in X. Only these potential matches
%  are passed to the algorithm for further scoring. 'WIDTH' is either a
%  two-element vector, [U, V] or a scalar that is used for both U and V.
%  Closeness is measured using only the first column (reference dimension)
%  in each data set. Default is Inf if 'BAND' is specified; otherwise
%  default is 10.
%
%  The 'WIDTH' constraint reduces the time and memory complexity of the
%  algorithm from O(MN) to O(sqrt(MN)*sqrt(UV)), where M and N are the
%  number of observations in X and Y respectively, and U and V are small
%  such that U<<M and V<<N.
%
%  If you specify both 'BAND' and 'WIDTH', only pairs of observations that
%  meet both constraints are considered potential matches and passed to the
%  dynamic programming algorithm for scoring.
%
%  Specify 'WIDTH' when you do not have a good estimate for the 'BAND'
%  property. To get an indication of the memory required to run the
%  algorithm with specific 'BAND' and 'WIDTH' parameters on your data sets,
%  run SAMPLEALIGN, but do not specify return values and set
%  'SHOWCONSTRAINTS' to true.
%
%  SAMPLEALIGN(...,'GAP',{G,H}) specifies the observation dependent terms
%  for assigning gap penalties. G is either a scalar or a function handle
%  specified using @(X), and H is either a scalar or a function handle
%  specified using @(Y). The functions @(X) and @(Y) must calculate the
%  penalty for each observation (row) when it is matched to a gap in the
%  other data set. The functions @(X) and @(Y) must return a column vector
%  with the same number of rows as X or Y, containing the gap penalty for
%  each observation (row). When 'GAP' is set either to a single scalar, or
%  a single function handle then the same value (or function handle) is
%  used for both G and H.
%
%  Gap penalties employed in the dynamic programming algorithm are computed
%  as follows. GPX is the gap penalty for matching observations from the
%  first data set X to gaps inserted in the second data set Y, and is the
%  product of two terms: GPX = G * QMS. The term G takes its value as a
%  function of the observations in X. With this, the user can introduce gap
%  penalties dependent on the reference column, dependent on other columns,
%  or, dependent on both. Similarly, GPY is the gap penalty for matching
%  observations from Y to gaps inserted in X, and is the product of two
%  terms: GPY = H * QMS. The term H takes its value as a function of the
%  observations in Y.
%
%  The term QMS is the 0.75 quantile of the score for the pairs of
%  observations that are potential matches (that is, pairs that comply with
%  the 'BAND' and 'WIDTH' constraints). If G and H are positive scalars,
%  then GPX and GPY are independent of the observation where the gap is
%  being inserted. Default 'GAP' is 1, that is, both G and H are 1, which
%  indicates that the default penalty for gap insertions in both sequences
%  is equivalent to the 0.75 quantile of the score for the pairs of
%  observations that are potential matches.
%
%  'GAP' defaults to a relatively safe value. However, the success of the
%  algorithm depends on the fine tuning of the gap penalties, which is
%  application dependent. When the gap penalties are large relative to the
%  score of the correct matches, samplealign returns alignments with fewer
%  gaps, but with more incorrectly aligned regions. When the gap penalties
%  are smaller, the output alignment contains longer regions with gaps and
%  fewer matched observations. Set 'SHOWNETWORK' to true to compare the gap
%  penalties to the score of matched observations in different regions of
%  the alignment.
%
%  SAMPLEALIGN(...,'QUANTILE',Q) specifies the quantile value used to
%  calculate the term QMS, which is used by the 'GAP' property to calculate
%  gap penalties. Q is a scalar between 0 and 1. Default is 0.75. Set Q to
%  an empty array ([]) to make the gap penalties independent of QMS, that
%  is GPX and GPY are functions of only the G and H input parameters
%  respectively.
%
%  SAMPLEALIGN(...,'DISTANCE',D) specifies a function to calculate the
%  distance between pairs of observations that are potential matches. D is
%  a function handle specified using @(R,S). The function @(R,S) must take
%  as arguments, R and S, matrices that have the same number of rows and
%  columns, and whose paired rows represent all potential matches of
%  observations in X and Y respectively. The function @(R,S) must return a
%  column vector of positive values with the same number of elements as
%  rows in R and S. Default is the Euclidean distance between the pairs.
%
%  By default All columns in X and Y, including the reference dimension,
%  are considered when calculating distances. If you do not want to include
%  the reference dimension in the distance calculations, use the 'WEIGHT'
%  property to exclude it.
%
%  SAMPLEALIGN(...,'WEIGHTS',W) controls the inclusion/exclusion of columns
%  (features) or the emphasis of columns (features) when calculating the
%  when calculating the distance score between observations that are
%  potential matches. W can be a logical row vector that specifies columns
%  in X and Y. W can also be a numeric row vector with the same number of
%  elements as columns in X and Y, that specifies the relative weights of
%  the columns (features). Default is a logical row vector with all
%  elements set to true.
%
%  Using a numeric row vector for W and setting some values to zero can
%  simplify the distance calculation when the data sets have many columns
%  (features).
%
%  The weight values are not considered when computing the constrained
%  alignment space, that is when using the 'BAND' or 'WIDTH' properties.
%
%  SAMPLEALIGN(...,'SHOWCONSTRAINTS',true) displays the search space
%  constrained by the input parameters 'BAND' and 'WIDTH', giving an
%  indication of the memory required to run the dynamic programming
%  algorithm. When you set 'SHOWCONSTRAINTS' to true, and do not specify
%  return values, SAMPLEALIGN does not run the dynamic programming
%  algorithm. This lets you explore the constrained space without running
%  into potential memory problems. 'SHOWCONSTRAINTS' defaults to false.
%
%  SAMPLEALIGN(...,'SHOWNETWORK',true) displays the dynamic programming
%  network, the match scores, the gap penalties, and the winning path (when
%  possible). 'SHOWNETWORK' defaults to false.
%
%  SAMPLEALIGN(...,'SHOWALIGNMENT',true) displays the first and second
%  columns of the X and Y data sets in the abscissa and the ordinate
%  respectively, of a two dimensional plot. Links between all the potential
%  matches that meet the constraints are displayed, and the matches
%  belonging to the output alignment are highlighted (when possible). Set
%  'SHOWALIGNMENT' to an integer to plot a different column of the inputs
%  in the ordinate. 'SHOWALIGNMENT' defaults to false.
%
%  Examples:
%
%     % Warp a sine wave with a smooth function such that it follows
%     % closely the cyclical sunspot activity, (a.k.a. Wolfer number):
%     load sunspot.dat
%     years = (1700:1990)';
%     T = 11.038; % approximate period (years)
%     f = @(y) 60 + 60 * sin(y*(2*pi/T));
%     [i,j] = samplealign([years f(years)],sunspot,'weights',[0 1],...
%                         'showalignment',true);
%     [p,s,mu] = polyfit(years(i),years(j),15);
%     wy = @(y) polyval(p,(y-mu(1))./mu(2));
%     years = (1700:1/12:1990)'; %plot warped signal monthly
%     figure
%     plot(sunspot(:,1),sunspot(:,2),years,f(years),wy(years),f(years))
%     legend('Sunspots','Unwarped Sine Wave','Warped Sine Wave')
%     title('Smooth Warping Example')
%
%     % Recover a non-linear warping between two signals that contain
%     % noisy Gaussian peaks:
%     peakLoc = [30 60 90 130 150 200 230 300 380 430];
%     peakInt = [7 1 3 10 3 6 1 8 3 10];
%     time = 1:450;
%     comp = exp(-(bsxfun(@minus,time,peakLoc')./5).^2);
%     sig_1 = (peakInt + rand(1,10)) * comp + rand(1,450);
%     sig_2 = (peakInt + rand(1,10)) * comp + rand(1,450);
%     wf = @(t) 1 + (t<=100).*0.01.*(t.^2) + (t>100).*(310+150*tanh(t./100-3));
%     sig_2 = interp1(time,sig_2,wf(time),'pchip');
%     [i,j] = samplealign([time;sig_1]',[time;sig_2]','weights',[0,1],...
%                          'band',35,'quantile',.5);
%     figure
%     sig_3 = interp1(time,sig_2,interp1(i,j,time,'pchip'),'pchip');
%     plot(time,sig_1,time,sig_2,time,sig_3)
%     legend('Reference','Distorted Signal','Corrected Signal')
%     title('Non-linear Warping Example')
%     figure
%     plot(time,wf(time),time,interp1(j,i,time,'pchip'))
%     legend('Distorting Function','Estimated Warping')
%
%  See also DIFFPROTDEMO, LCMSDEMO, MSALIGN, MSHEATMAP, MSPALIGN,
%  MSPPRESAMPLE, MSPREPRODEMO, MSRESAMPLE.

%  References:
%
%  [1] C. S. Myers and L. R. Rabiner. A comparative study of several
%      dynamic time-warping algorithms for connected word recognition. The
%      Bell System Technical Journal, 60(7):1389-1409, Sept. 1981.
%  [2] H. Sakoe and S. Chiba. Dynamic programming algorithm optimization
%      for spoken word recognition. IEEE Trans. Acoustics, Speech and
%      Signal Processing, ASSP-26(1):43-49, Feb. 1978.

%  Copyright 2007-2008 The MathWorks, Inc.
%  $Revision: 1.1.6.11 $  $Date: 2008/06/24 16:59:51 $

%%% Validate mandatory inputs %%%
[nX,nD] = size(X);
[nY,n]  = size(Y);

if ~isnumeric(X) || ~isnumeric(Y) || ~isreal(X) || ~isreal(Y)
    error('Bioinfo:samplealign:illegalType','X and Y must be real and numeric.')
end
if n~=nD || nX<2 || nY<2  || n==0
    error('Bioinfo:samplealign:badSize','X and Y must have the same number of columns and at least 2 rows.')
end

%%% Optional input PPV parsing %%%
[band,bandIsDefault,widthX,widthY,widthIsDefault,gapX,gapY, ...
    distance,weights, showConstraints, showAlignment,showNetwork, ...
    quantileValue,computeQuantile,axisLink] = parse_inputs(nD,varargin{:});

%%% Initialize some figure handles
fhAli=0;fhCon1=0;fhCon2=0;fhNet=0;

%%% Get and check the reference vectors (first column of the inputs) %%%
x = double(X(:,1));
y = double(Y(:,1));
if ~issorted(x) || ~issorted(y) || isnan(x(end)) || isnan(y(end)) || isinf(x(end)) || isinf(y(end))
    error('Bioinfo:samplealign:invalidReferenceDimension',...
        ['The first column in X and Y must be sorted in\n',...
        ' ascending order, and cannot contain NaNs or Infs.'])
end
if showAlignment % save dimensions that are to be plotted in the showAlignment figure
    xa = X(:,showAlignment);
    ya = Y(:,showAlignment);
end

%%% Calculate position dependent gap terms %%%
if isa(gapX,'function_handle')
    gapXf = gapX;
    try
        gapX = gapXf(X);
    catch theException
        error('Bioinfo:samplealign:ErrorInUserGapXFunction',...
            ['The gap function ''%s'' generated the '...
            'following error:\n%s'], func2str(gapXf),theException.message);
    end
    if ~isnumeric(gapX) || ~(isscalar(gapX) || (isvector(gapX)&&(size(gapX,1)==nX)))
        error('Bioinfo:samplealign:UserGapXFunctionBadReturn',...
            'The gap function ''%s'' returned a invalid output.',...
            func2str(gapXf));
    end
end
if isa(gapY,'function_handle')
    gapYf = gapY;
    try
        gapY = gapYf(Y);
    catch theException
        error('Bioinfo:samplealign:ErrorInUserGapYFunction',...
            ['The gap function ''%s'' generated the '...
            'following error:\n%s'], func2str(gapYf),theException.message);
    end
    if ~isnumeric(gapY) || ~(isscalar(gapY) || (isvector(gapY)&&(size(gapY,1)==nY)))
        error('Bioinfo:samplealign:UserGapYFunctionBadReturn',...
            'The gap function ''%s'' returned a invalid output.',...
            func2str(gapYf));
    end
end

%%% Reduce and weight observations, used later for computing distances %%%
if ~all(weights)
    h = weights>0;
    X = X(:,h);
    Y = Y(:,h);
    weights = weights(h);
    if isempty(weights)
        error('Bioinfo:samplealign:tooFewDimensions',...
            ['At least one column of the inputs must be considered\n',...
            'for computing the distances, correct the WEIGHTS input.'])
    end
end
if ~islogical(weights)
    X = bsxfun(@times,X,weights');
    Y = bsxfun(@times,Y,weights');
end
if any(any(isnan(X))|any(isinf(X))|any(isnan(Y))|any(isinf(Y)))
    error('Bioinfo:samplealign:nansorinfsInInputs',...
        ['There are NaNs or Infs in the columns of the inputs\n',...
        'X and Y that are considered for computing the distances.'])
end

%%% Find the likely matches that comply with the BAND constraint %%%
h = zeros(nX,1); % contains first valid y for each x
g = zeros(nX,1); % contains last valid y for each x
up=1; bot=1;
if isa(band,'function_handle')
    try
        for i = 1:nX
            while (up<=nY) && (x(i)-y(up) > band((x(i)+y(up))/2))
                up = up+1;
            end
            while (bot<=nY) && (y(bot)-x(i) <= band((y(bot)+x(i))/2))
                bot = bot+1;
            end
            h(i) = up;
            g(i) = bot-1;
        end
    catch topErr
        try
            ev1 = band((x(i)+y(up))/2);
        catch theException
            error('Bioinfo:samplealign:ErrorInBandFunction',...
                ['The band function ''%s'' generated the '...
                'following error when evaluated with ''%f'':\n%s'],...
                func2str(band),(x(i)+y(up))/2,theException.message);
        end
        try
            ev2 = band((y(bot)+x(i))/2);
        catch theException
            error('Bioinfo:samplealign:ErrorInBandFunction',...
                ['The band function ''%s'' generated the '...
                'following error when evaluated with ''%f'':\n%s'],...
                func2str(band),(y(bot)+x(i))/2,theException.message);
        end
        if ~isscalar(ev1) || ~isscalar(ev2)
            error('Bioinfo:samplealign:BandFunctionReturnsIncorrectSize',...
                'The band function ''%s'' returned a non scalar value',func2str(band))
        end
        rethrow(topErr)
    end
elseif band>(max(x(end),y(end))-min(x(1),y(1)))
    h(:) = 1;
    g(:) = nY;
else
    for i = 1:nX
        while (up<=nY) && (x(i)-y(up) > band)
            up = up+1;
        end
        while (bot<=nY) && (y(bot)-x(i) <= band)
            bot = bot+1;
        end
        h(i) = up;
        g(i) = bot-1;
    end
end

%%% Find the likely matches that comply with the WIDTH constraints %%%
if widthX>=nX || widthY>=nY % each x gets all the samples in y
    c = ones(nX,1);
    d = nY(ones(nX,1));
else
    % find the first and last valid x for each y and put them in bs and be
    keps = mean(diff(x))/1e10;
    t = sortrows([filter(accumarray([1;widthX+1],[1 1],[widthX+1 1]),2,x)+keps ones(nX,1);y zeros(nY,1)])*[0;1];
    ct =  cumsum(t);
    bs = max(1,ct(~t)-widthX+1);
    r = 1;
    while r
        r = find((x(bs)==x(max(1,bs-1)))&(bs>1));
        bs(r) = bs(r)-1;
    end
    be = max(widthX+1,ct(~t));
    r = 1;
    while r
        r = find((max(y-x(bs),x(be)-y)>=x(min(nX,be+1))-y-keps)&(be<nX));
        be(r) = be(r)+1;
    end
    
    % find the first and last valid y for each x and put them in c and d
    keps = mean(diff(y))/1e10;
    t = sortrows([filter(accumarray([1;widthY+1],[1 1],[widthY+1 1]),2,y)+keps ones(nY,1);x zeros(nX,1)])*[0;1];
    ct =  cumsum(t);
    c = max(1,ct(~t)-widthY+1);
    r = 1;
    while r
        r = find((y(c)==y(max(1,c-1)))&(c>1));
        c(r) = c(r)-1;
    end
    d = max(widthY,ct(~t));
    r = 1;
    while r
        r = find((max(x-y(c),y(d)-x)>=y(min(nY,d+1))-x-keps)&(d<nY));
        d(r) = d(r)+1;
    end
    
    % Coalesce bs and be into c and d using an 'or' logic
    be = cumsum(accumarray(be(be<nX),1,[nX 1]))+1;
    bs = cumsum(accumarray(bs,1,[nX 1]));
    t = be<bs;
    c(t) = min(be(t),c(t));
    d(t) = max(bs(t),d(t));
end

if showConstraints
    fhCon1 = figure; hold on; axis equal
    set(gca,'xlimmode','manual','ylimmode','manual')
    yy = bsxfun(@plus,[0 h'-1 nY nY g(end:-1:1)' 0],[0.5;0.5]);
    zz = bsxfun(@plus,[0 c'-1 nY nY d(end:-1:1)' 0],[0.5;0.5]);
    xx = [bsxfun(@plus,0:nX+1,[-0.5;0.5]) bsxfun(@plus,nX+1:-1:0,[0.5;-0.5])];
    
    %%% Set the color scheme for the plot.
    %%% Note that we use transparency to display the overlapping regions so
    %%% in order to get the legend color correct we must calculate the RGB
    %%% values that are displayed.
    
    bandColor = [1 0 0];
    widthColor = [0 0 1 ] ;
    falpha = .5 ;
    whiteColor = [1 1 1];
    legendColor = falpha*bandColor + (1-falpha)*whiteColor;
    intersectColor = falpha*widthColor + (1-falpha)*bandColor;
    
    %%% End of color setup
    
    hp1 = patch(xx(:),yy(:),bandColor,'facealpha',falpha,'EdgeColor','none'); %#ok<NASGU>
    hp2 = patch(xx(:),zz(:),widthColor,'EdgeColor','none');
    % Create two dummy patches so that we can set the legend color
    % correctly.
    hp3 = patch([0 0 -1],[0 -1 0],intersectColor,'EdgeColor',intersectColor);
    hp4 = patch([0 0 -1],[0 -1 0],legendColor,'EdgeColor',legendColor);
    xlabel('Index of input X')
    ylabel('Index of input Y')
    title('Constraints in the Index Space')
    legend([hp4,hp2,hp3],{'Band','Width','Combined'},'Location','BestOutside')
    axis([.5 nX+.5 0.5 nY+.5])
    if axisLink
        setupSamplealignListeners(@indexConsChanged,@indexConsChanged)
    end
    fhCon2 = figure; hold on; axis equal
    set(gca,'xlimmode','manual','ylimmode','manual')
    hp1 = patch(interp1(1:nX,x,xx(:),'pchip'),interp1(1:nY,y,yy(:),'pchip'),bandColor,'EdgeColor','none','facealpha',falpha); %#ok<NASGU>
    hp2 = patch(interp1(1:nX,x,xx(:),'pchip'),interp1(1:nY,y,zz(:),'pchip'),widthColor,'EdgeColor','none');
    % Create two dummy patches so that we can set the legend color
    % correctly.
    hp3 = patch(interp1(1:nX,x,[0 0 -1],'pchip'),interp1(1:nY,y,[0 -1 0],'pchip'),intersectColor,'EdgeColor',intersectColor);
    hp4 = patch([0 0 -1],[0 -1 0],legendColor,'EdgeColor',legendColor);
    xlabel('Reference dimension of input X')
    ylabel('Reference dimension of input Y')
    title('Constraints in the Reference Dimension')
    legend([hp4,hp2,hp3],{'Band','Width','Combined'},'Location','BestOutside')
    axis([interp1(1:nX,x,[.5 nX+.5],'pchip') interp1(1:nY,y,[0.5 nY+.5],'pchip')])
    if axisLink
        setupSamplealignListeners(@referenceConsChanged,@referenceConsChanged)
    end
    % Coalesce c and d into h and g using an 'and' logic
    h = max(h,c);
    g = min(g,d);
    xf = g>=h; % values in h and g that represent rows with valid matches
    nM = sum(g(xf)-h(xf)+1); % number of matches
    figure(fhCon1)
    text(min(xlim),max(ylim),sprintf('  Number of nodes: %d',nM),'Vertical','Top')
    figure(fhCon2)
    text(min(xlim),max(ylim),sprintf('  Number of nodes: %d',nM),'Vertical','Top')
    if ~showAlignment && ~showNetwork && nargout==0
        % we allow the user to exit the function early without finishing
        % the algorithm, because if constraints are not appropriate, it's
        % very easy to run out of memory, by this the user can first assure
        % that the problem is trackable before running out of memory.
        if axisLink
            setupSampleAlignAppData([fhCon1,fhCon2,fhNet,fhAli],x,y)
        end
        return
    end
end

%%% Coalesce c and d into h and g, i.e. intersect constrained spaces %%%
h = max(h,c);
g = min(g,d);
clear be bs ct c d t % some memory clean up before going on

%%% Find nodes that represent matched observations %%%
xf = g>=h; % values in h and g that represent rows with valid matches
nM = double(sum(g(xf)-h(xf)+1)); % number of matches
mx = zeros(nM,1,'uint32');  % list of matches in X
my = zeros(nM,1,'uint32');  % list of matches in Y
mx(cumsum(g(xf)-h(xf)+1)) = find(xf);
my(cumsum(g(xf)-h(xf)+1)) = g(xf);
for i=nM:-1:1
    if ~mx(i)
        mx(i) = mx(i+1);
        my(i) = my(i+1)-1;
    end
end

%%% Find nodes that need to be added to join unconnected components %%%
vg = g.*~xf+(h-1).*xf;                       % filling vertical gaps
hg = cumsum(accumarray(g(g<nY)+1,1,[nY,1])); % filling horizontal gaps

%%% Create a hash table to easy recover nodes in the graph %%
numNodes = nM+nX+nY+1;
numEdges = 2*numel(my)+numNodes;
nodes = sortrows([0 0;my mx;vg (1:nX)';(1:nY)' hg]);
hT = sparse(double(nodes(:,1).*(nX+1)+nodes(:,2)+1),...
    1,1:numNodes,(nX+1)*(nY+1),1,numNodes);

%%% Score the matches (only if needed) %%%
% Store distances only if we'll use them later otherwise we'll plug them
% directly to the graph later in order to save memory space
if computeQuantile || (showNetwork && numEdges<=500000)
    bSize = floor((12500000/size(X,2))); % bSize in rows
    matchDistances = zeros(nM,1);
    for i = 0:bSize:nM
        j = min(i+bSize,nM);
        try
            tD = distance(X(mx(i+1:j),:),Y(my(i+1:j),:));
        catch theException
            error('Bioinfo:samplealign:ErrorInDistanceFunction',...
                ['The distance function ''%s'' generated the '...
                'following error:\n%s'], func2str(distance),theException.message);
        end
        if ~isnumeric(tD) || ~(isscalar(tD) || (isvector(tD)&&(size(tD,1)==(j-i))))
            error('Bioinfo:samplealign:DistanceFunctionBadReturn',...
                'The distance function ''%s'' returned a invalid output.',...
                func2str(distance));
        end
        matchDistances(i+1:j) = tD;
    end
    % making sure there are not zero valued or negative edges
    matchDistances = max(matchDistances,eps);
end

%%% Adjust gap penalties by QMS when needed: %%%
if computeQuantile
    if numNodes<500000  % use stats quantile...
        QMS = quantile(matchDistances,quantileValue);
    else % use a quick quantile approx which uses accumarray...
        QMS = myQuantile(matchDistances,quantileValue,10000);
    end
    gapX = gapX.*double(QMS);
    gapY = gapY.*double(QMS);
end

%%% making sure there are not zero valued or negative edges %%%
gapX = max(eps,gapX);
gapY = max(eps,gapY);

if showNetwork
    fhNet = figure; hold on; axis equal
    set(gca,'xlimmode','manual','ylimmode','manual')
    if numEdges>1000000
        warning('Bioinfo:samplealign:TooManyEdgesForGraph',...
            ['There are more than 1000000 edges, \n'...
            'edges are not displayed in the graph.'])
        if nargout==0
            warning('Bioinfo:samplealign:TooManyEdgesForGraphExtra',...
                ['Also, SAMPLEALIGN was called without output arguments, \n',...
                'then the dynamic programming algorithm does not run and \n',...
                'the winning path is neither displayed.'])
        end
    elseif numEdges>500000
        warning('Bioinfo:samplealign:TooManyEdgesForColoredGraph',...
            ['There are more than 500000 edges, \n'...
            'edges are not colored by its score.'])
        if nargout==0
            warning('Bioinfo:samplealign:TooManyEdgesForColoredGraphExtra',...
                ['Also, SAMPLEALIGN was called without output arguments, \n',...
                'then the dynamic programming algorithm does not run and \n',...
                'the winning path is neither displayed.'])
        end
        ind = reshape(repmat(1:nM,3,1),nM*3,1);
        delta = reshape([ones(1,nM);zeros(1,nM);nan(1,nM)],nM*3,1);
        
        % draw edges for matches
        plot(double(mx(ind))-delta,double(my(ind))-delta,'b-','LineWidth',3)
        % draw edges for vertical gaps that lead to match nodes
        plot(mx(ind),double(my(ind))-delta,'g-','LineWidth',3)
        % draw edges for horizontal gaps that lead to match nodes
        plot(double(mx(ind))-delta,my(ind),'c-','LineWidth',3)
        % draw edges for all other vertical gaps
        ind = reshape(repmat(1:nY,3,1),nY*3,1);
        delta = reshape([ones(1,nY);zeros(1,nY);nan(1,nY)],nY*3,1);
        plot(hg(ind),ind-delta,'g-','LineWidth',3)
        % draw edges for all other horizontal gaps
        ind = reshape(repmat(1:nX,3,1),nX*3,1);
        delta = reshape([ones(1,nX);zeros(1,nX);nan(1,nX)],nX*3,1);
        plot(ind-delta,vg(ind),'c-','LineWidth',3)
        
    else
        ind = reshape(repmat(1:nM,3,1),nM*3,1);
        delta = reshape([ones(1,nM);zeros(1,nM);nan(1,nM)],nM*3,1);
        
        % draw edges for matches
        patch(double(mx(ind))-delta,double(my(ind))-delta,matchDistances(ind),...
            'LineWidth',3,'EdgeColor','flat','FaceColor','none','VertexNormals',[])
        
        % draw edges for vertical gaps that lead to match nodes
        if isscalar(gapY)
            gapY = gapY(ones(nY,1));
        end
        patch(mx(ind),double(my(ind))-delta,gapY(my(ind)),...
            'LineWidth',3,'EdgeColor','flat','FaceColor','none','VertexNormals',[])
        % draw edges for horizontal gaps that lead to match nodes
        if isscalar(gapX)
            gapX = gapX(ones(nX,1));
        end
        patch(double(mx(ind))-delta,my(ind),gapX(mx(ind)),...
            'LineWidth',3,'EdgeColor','flat','FaceColor','none','VertexNormals',[])
        % draw edges for all other vertical gaps
        ind = reshape(repmat(1:nY,3,1),nY*3,1);
        delta = reshape([ones(1,nY);zeros(1,nY);nan(1,nY)],nY*3,1);
        patch(hg(ind),ind-delta,gapY(ind),...
            'LineWidth',3,'EdgeColor','flat','FaceColor','none','VertexNormals',[])
        % draw edges for all other horizontal gaps
        ind = reshape(repmat(1:nX,3,1),nX*3,1);
        delta = reshape([ones(1,nX);zeros(1,nX);nan(1,nX)],nX*3,1);
        patch(ind-delta,vg(ind),gapX(ind),...
            'LineWidth',3,'EdgeColor','flat','FaceColor','none','VertexNormals',[])
        ylabel(colorbar,'Gap and Match Scores')
    end
    % draw nodes
    plot([0;(1:nX)';hg],[0;vg;(1:nY)'],'ko','MarkerFaceColor','w')
    plot(mx,my,'ks','MarkerFaceColor','w')
    axis([-.5 nX+.5 -0.5 nY+.5])
    if axisLink
        setupSamplealignListeners(@netChanged,@netChanged)
    end
    xlabel('Index of input X')
    ylabel('Index of input Y')
    title('Dynamic Programming Network')
    if ~showAlignment && nargout==0
        % we allow the user to exit the function early without finishing
        % the algorithm, because if constraints are not appropriate, it's
        % very easy to run out of memory, by this the user can first assure
        % that the problem is trackable before running out of memory.
        if axisLink
            setupSampleAlignAppData([fhCon1,fhCon2,fhNet,fhAli],x,y)
        end
        return
    end
end

if showAlignment
    fhAli = figure; hold on
    set(gca,'xlimmode','manual','ylimmode','manual')
    if numNodes>100000
        warning('Bioinfo:samplealign:TooManyNodesForGraph',...
            ['There are more than 100000 potential matches between \n',...
            'observations, potential matches are not displayed in the graph.'])
        if nargout==0
            warning('Bioinfo:samplealign:TooManyNodesForGraphExtra',...
                ['Also, SAMPLEALIGN was called without output arguments, \n',...
                'then the dynamic programming algorithm does not run and \n',...
                'matches that belong to the winning path are neither displayed.'])
        end
    else
        hlPotMat = plot([x(mx) y(my)]',[xa(mx) ya(my)]','color',[.8 .8 .8]);
    end
    hlPoints(2,1) = plot(x,xa,'bo','MarkerSize',2,'linewidth',2);
    hlPoints(1,1) = plot(y,ya,'o','color',[0 .5 0],'MarkerSize',2,'linewidth',2);
    axis([min(min(x),min(y)) max(max(x),max(y)) min(min(xa),min(ya)) max(max(xa),max(ya))])
    if nX>500 || nY>500
        warning('Bioinfo:samplealign:TooManyObservationsForGraph',...
            ['At least one of the input signals has more than\n',...
            '500 observations, indices are not displayed in the graph.'])
    else
        text(x,xa,num2str((1:nX)'),'color','b','fontsize',8,'clipping','on','verticalalignment','top')
        text(y,ya,num2str((1:nY)'),'color',[0 .5 0],'fontsize',8,'clipping','on','verticalalignment','bottom')
    end
    xlabel('Reference Dimension')
    ylabel(sprintf('Dimension %d',showAlignment))
    if axisLink
        setupSamplealignListeners(@alignChanged)
    end
    title('Sample Matching')
    str1 = sprintf('Samples of input X (%d)',nX);
    str2 = sprintf('Samples of input Y (%d)',nY);
    str3 = sprintf('Potential Matches (%d)',nM);
    if numNodes>100000 || nM==0
        legend(hlPoints,{str1,str2})
    else
        legend([hlPoints;hlPotMat(1)],{str1,str2,str3})
    end
    if nargout==0
        % we allow the user to exit the function early without finishing
        % the algorithm, because if constraints are not appropriate, it's
        % very easy to run out of memory, by this the user can first assure
        % that the problem is trackable before running out of memory.
        if axisLink
            setupSampleAlignAppData([fhCon1,fhCon2,fhNet,fhAli],x,y)
        end
        return
    end
end

%%% Create the graph with all the gap penalties %%%
if isscalar(gapX) && isscalar(gapY) && gapX==gapY
    graph = sparse(hT([my-1;(0:nY-1)';my;vg].*(nX+1)+[mx;hg;mx-1;(0:nX-1)']+1),...
        hT([my;  (1:nY)';  my;vg].*(nX+1)+[mx;hg;mx;  (1:nX)']  +1),...
        gapX,numNodes,numNodes,numEdges);
else
    if isscalar(gapX)
        gapX = gapX(ones(nX,1));
    end
    if isscalar(gapY)
        gapY = gapY(ones(nY,1));
    end
    graph = sparse(hT([my-1;(0:nY-1)';my;vg].*(nX+1)+[mx;hg;mx-1;(0:nX-1)']+1),...
        hT([my;  (1:nY)';  my;vg].*(nX+1)+[mx;hg;mx;  (1:nX)']  +1),...
        [gapY([my;(1:nY)']);gapX([mx;(1:nX)'])],...
        numNodes,numNodes,numEdges);
end

%%% Insert the match scores into the previous graph %%%
if computeQuantile || (showNetwork && numEdges<=500000) % match distances are already computed?
    graph = graph + sparse(hT((my-1).*(nX+1)+(mx-1)+1),hT(my.*(nX+1)+mx+1),...
        double(matchDistances),numNodes,numNodes,nM);
else
    bSize = floor((12500000/size(X,2))); % bSize in rows
    for i = 0:bSize:nM
        j = min(i+bSize,nM);
        try
            tD = distance(X(mx(i+1:j),:),Y(my(i+1:j),:));
        catch theException
            error('Bioinfo:samplealign:ErrorInDistanceFunction',...
                ['The distance function ''%s'' generated the '...
                'following error:\n%s'], func2str(distance),theException.message);
        end
        if ~isnumeric(tD) || ~(isscalar(tD) || (isvector(tD)&&(size(tD,1)==(j-i))))
            error('Bioinfo:samplealign:DistanceFunctionBadReturn',...
                'The distance function ''%s'' returned a invalid output.',...
                func2str(distance));
        end
        graph = graph + sparse(hT((my(i+1:j)-1).*(nX+1)+(mx(i+1:j)-1)+1),...
            hT(my(i+1:j).*(nX+1)+mx(i+1:j)+1),...
            double(max(eps,tD)),numNodes,numNodes);
    end
end

%%% Solve graph shortest path problem %%%%
[d,path] = graphshortestpath(graph,1,numNodes);
clear graph hashTable % some memory clean up before continuing

%%% Draw winning path on the Dynamic Programming Network %%%
if showNetwork
    figure(fhNet)
    plot(nodes(path,2),nodes(path,1),'r:.','MarkerSize',10)
    title('Dynamic Programming Network and Shortest Path')
end

%%% Setting up and checking the outputs %%%
j = nodes(path,1);
i = nodes(path,2);
k = find((diff(i)==1) & (diff(j)==1));
if isempty(k)
    if nM
        warning('Bioinfo:samplealign:NoMatchesFound',...
            ['No observations could be matched, the winning path\n',...
            'contains only gap insertions. Are gap penalties too small?'])
    else
        warning('Bioinfo:samplealign:NoPossibleMatchesFound',...
            ['BAND constrain leads to no potential observations to be matched.\n',...
            'Do inputs X(:,1) and Y(:,1) share the same range or is the ''BAND'' parameter too small?'])
    end
end
I = double(i(k+1));
J = double(j(k+1));

%%% Draw the aligned signals %%%
if showAlignment
    figure(fhAli)
    hlMatches = plot([x(I) y(J)]',[xa(I) ya(J)]','color','r');
    str4 = sprintf('Selected Matches (%d)',numel(k));
    if numNodes>100000 || nM==0
        if isempty(k)
            legend(hlPoints,{str1,str2})
        else
            legend([hlPoints;hlMatches(1)],{str1,str2,str4})
        end
    elseif isempty(k)
        legend([hlPoints;hlPotMat(1)],{str1,str2,str3})
    else
        legend([hlPoints;hlPotMat(1);hlMatches(1)],{str1,str2,str3,str4})
    end
end

if axisLink
    setupSampleAlignAppData([fhCon1,fhCon2,fhNet,fhAli],x,y)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function th = myQuantile(P,quan_int,res_int)
% MYQUANTILE computes a quick coarse quantile using accumarray
imi = min(P);
ima = max(P);
in2idx = @(x,r) round((x - imi) / (ima-imi) * (r-1) + .5);
idx2in = @(x,r) (x-.5) / (r-1) * (ima-imi) + imi;
inva = accumarray(in2idx(P,res_int),1,[res_int 1]);
th = idx2in(interp1q(cumsum(inva)/sum(inva),(1:res_int)',quan_int),res_int);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function setupSampleAlignAppData(fhs,x,y)
% helper function to set up appdata used in figure listeners
arrayfun(@(fh) setappdata(fh,'samplealignFigureHandles',fhs),nonzeros(fhs))
arrayfun(@(fh) setappdata(fh,'samplealignReferenceX',x),nonzeros(fhs))
arrayfun(@(fh) setappdata(fh,'samplealignReferenceY',y),nonzeros(fhs))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function setupSamplealignListeners(fhX,fhY)
% helper function to set up listeners and X and Y axes, pass empty to do
% not set a listener
hgp     = findpackage('hg');
axesC   = findclass(hgp,'axes');
% listens when the Ylim of axes has changed
if nargin>1 && ~isempty(fhY)
    YLimListener = handle.listener(gca,axesC.findprop('YLim'),...
        'PropertyPostSet',{fhY,gcf,gca});
else
    YLimListener = [];
end
% listens when the Xlim of axes has changed
if ~isempty(fhX)
    XLimListener = handle.listener(gca,axesC.findprop('XLim'),...
        'PropertyPostSet',{fhX,gcf,gca});
else
    XLimListener = [];
end
% store the listeners in current figure appdata
setappdata(gcf,'samplealignListeners',[YLimListener, XLimListener]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function switchSamplealignListeners(fh,str)
for i = 1:4
    if ishandle(fh(i)) && fh(i)>0
        set(getappdata(fh(i),'samplealignListeners'),'Enabled',str)
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function indexConsChanged(hSrc,event,hf,ha) %#ok
fh = getappdata(hf,'samplealignFigureHandles');
if any(ishandle(fh(2:4))& fh(2:4)>0)
    switchSamplealignListeners(fh,'off')
    x = getappdata(hf,'samplealignReferenceX');
    y = getappdata(hf,'samplealignReferenceY');
    if ishandle(fh(2)) && fh(2)>0
        ta = findobj(get(fh(2),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(1:numel(x),x,get(ha,'Xlim'),'pchip'))
            ylim(ta,interp1(1:numel(y),y,get(ha,'Ylim'),'pchip'))
        end
    end
    if ishandle(fh(3)) && fh(3)>0
        ta = findobj(get(fh(3),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,get(ha,'Xlim')-[1 0])
            ylim(ta,get(ha,'Ylim')-[1 0])
        end
    end
    if ishandle(fh(4)) && fh(4)>0
        ta = findobj(get(fh(4),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xl = interp1(1:numel(x),x,get(ha,'Xlim'),'pchip');
            yl = interp1(1:numel(y),y,get(ha,'Ylim'),'pchip');
            xlim(ta,[min(xl(1),yl(1)) max(xl(2),yl(2))])
        end
    end
    switchSamplealignListeners(fh,'on')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function referenceConsChanged(hSrc,event,hf,ha) %#ok
fh = getappdata(hf,'samplealignFigureHandles');
if any(ishandle(fh([1 3 4]))& fh([1 3 4])>0)
    switchSamplealignListeners(fh,'off')
    x = getappdata(hf,'samplealignReferenceX');
    y = getappdata(hf,'samplealignReferenceY');
    if ishandle(fh(1)) && fh(1)>0
        ta = findobj(get(fh(1),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(x,1:numel(x),get(ha,'Xlim'),'pchip'))
            ylim(ta,interp1(y,1:numel(y),get(ha,'Ylim'),'pchip'))
        end
    end
    if ishandle(fh(3)) && fh(3)>0
        ta = findobj(get(fh(3),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(x,1:numel(x),get(ha,'Xlim'),'pchip')-[1 0])
            ylim(ta,interp1(y,1:numel(y),get(ha,'Ylim'),'pchip')-[1 0])
        end
    end
    if ishandle(fh(4)) && fh(4)>0
        ta = findobj(get(fh(4),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xl = xlim;
            yl = ylim;
            xlim(ta,[min(xl(1),yl(1)) max(xl(2),yl(2))])
        end
    end
    switchSamplealignListeners(fh,'on')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function netChanged(hSrc,event,hf,ha) %#ok
fh = getappdata(hf,'samplealignFigureHandles');
if any(ishandle(fh([1 2 4]))& fh([1 2 4])>0)
    switchSamplealignListeners(fh,'off')
    x = getappdata(hf,'samplealignReferenceX');
    y = getappdata(hf,'samplealignReferenceY');
    if ishandle(fh(1)) && fh(1)>0
        ta = findobj(get(fh(1),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,get(ha,'Xlim')+[1 0])
            ylim(ta,get(ha,'Ylim')+[1 0])
        end
    end
    if ishandle(fh(2)) && fh(2)>0
        ta = findobj(get(fh(2),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(1:numel(x),x,get(ha,'Xlim')+[1 0],'pchip'))
            ylim(ta,interp1(1:numel(y),y,get(ha,'Ylim')+[1 0],'pchip'))
        end
    end
    if ishandle(fh(4)) && fh(4)>0
        ta = findobj(get(fh(4),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xl = interp1(1:numel(x),x,get(ha,'Xlim')+[1 0],'pchip');
            yl = interp1(1:numel(y),y,get(ha,'Ylim')+[1 0],'pchip');
            xlim(ta,[min(xl(1),yl(1)) max(xl(2),yl(2))])
        end
    end
    switchSamplealignListeners(fh,'on')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function alignChanged(hSrc,event,hf,ha) %#ok
fh = getappdata(hf,'samplealignFigureHandles');
if any(ishandle(fh([1 2 3]))& fh([1 2 3])>0)
    switchSamplealignListeners(fh,'off')
    x = getappdata(hf,'samplealignReferenceX');
    y = getappdata(hf,'samplealignReferenceY');
    if ishandle(fh(1)) && fh(1)>0
        ta = findobj(get(fh(1),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(x,1:numel(x),get(ha,'Xlim'),'pchip'))
            ylim(ta,interp1(y,1:numel(y),get(ha,'Xlim'),'pchip'))
        end
    end
    if ishandle(fh(2)) && fh(2)>0
        ta = findobj(get(fh(2),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(1:numel(x),x,interp1(x,1:numel(x),get(ha,'Xlim'),'pchip'),'pchip'))
            ylim(ta,interp1(1:numel(y),y,interp1(y,1:numel(y),get(ha,'Xlim'),'pchip'),'pchip'))
        end
    end
    if ishandle(fh(3)) && fh(3)>0
        ta = findobj(get(fh(3),'Children'),'Type','axes','Tag','');
        if numel(ta)==1
            xlim(ta,interp1(x,1:numel(x),get(ha,'Xlim')-[1 0],'pchip'))
            ylim(ta,interp1(y,1:numel(y),get(ha,'Xlim')-[1 0],'pchip'))
        end
    end
    switchSamplealignListeners(fh,'on')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [band,bandIsDefault,widthX,widthY,widthIsDefault,gapX,gapY, ...
    distance,weights, showConstraints, showAlignment,showNetwork, ...
    quantileValue,computeQuantile,axisLink] = parse_inputs(nD,varargin)

% Parse the varargin parameter/value inputs

% Check that we have the right number of inputs
if rem(nargin,2)== 0
    error(sprintf('Bioinfo:%s:IncorrectNumberOfArguments',mfilename),...
        'Incorrect number of arguments to %s.',mfilename);
end

% The allowed inputs
okargs = {'band','width','gap','distance','weights',...
    'showconstraints','constraints',...
    'showalignment','alignment',...
    'shownetwork','network','quantile','linkaxes'};

%%% Set defaults for input parameters %%%
band = inf;
bandIsDefault = true;
widthIsDefault = true;
quantileValue = 0.75;
computeQuantile = true;

distance = @(R,S) sqrt(sum((R-S).^2,2));
gapX = 1;
gapY = 1;
weights = true(nD,1);

showAlignment = false;
showConstraints = false;
showNetwork = false;

axisLink = true;

% Loop over the values

for j=1:2:nargin-1
    % Lookup the pair
    [k, pval] = pvpair(varargin{j}, varargin{j+1}, okargs, mfilename);
    switch(k)
        
        case 1 % BAND
            if isa(pval,'function_handle')
                band = pval;
                bandIsDefault = false;
            elseif isscalar(pval) && pval>0
                band = pval;
                bandIsDefault = false;
            else
                error('Bioinfo:samplealign:InvalidBand',...
                    'BAND must be a positive scalar or a function handle in the form of @(z).')
            end
        case 2 % WIDTH
            if isvector(pval) && numel(pval)==2 && all(pval>0)
                widthX = pval(1);
                widthY = pval(2);
                widthIsDefault = false;
            elseif isscalar(pval) && pval>0
                widthX = pval;
                widthY = pval;
                widthIsDefault = false;
            else
                error('Bioinfo:samplealign:InvalidWidth',...
                    'WIDTH must be a positive scalar or a vector with two positive values.')
            end
        case 3 % GAP
            if isscalar(pval) && (isnumeric(pval)||isa(pval,'function_handle'))
                % undocumented handy option
                gapX = pval;
                gapY = pval;
            elseif numel(pval)==2 && isnumeric(pval)
                % undocumented handy option
                gapX = pval(1);
                gapY = pval(2);
            elseif iscell(pval) && any(numel(pval)==[1 2])
                % documented correct syntax
                if numel(pval)==1;
                    pval = pval([1 1]);
                end
                if isa(pval{1},'function_handle')
                    gapX = pval{1};
                elseif isnumeric(pval{1}) && isscalar(pval{1})
                    gapX = pval{1};
                else
                    error('Bioinfo:samplealign:InvalidGapX',...
                        'The first element in GAP must be a function handle or a scalar.')
                end
                if isa(pval{2},'function_handle')
                    gapY = pval{2};
                elseif isnumeric(pval{2}) && isscalar(pval{2})
                    gapY = pval{2};
                else
                    error('Bioinfo:samplealign:InvalidGapY',...
                        'The second element in GAP must be a function handle or a scalar.')
                end
            else
                error('Bioinfo:samplealign:InvalidSizeGap',...
                    'GAP must be a cell array with one or two elements.')
            end
            
        case 4 % DISTANCE
            if isa(pval,'function_handle')
                distance = pval;
            else
                error('Bioinfo:samplealign:InvalidDistance',...
                    'DISTANCE must be a function handle.')
            end
        case 5 % WEIGHTS
            if isvector(pval) && numel(pval)==nD;
                weights = pval(:);
            else
                error('Bioinfo:samplealign:InvalidWeights',...
                    'WEIGHTS must be a vector with the same number of elements as columns in X and Y')
            end
        case {6,7} % SHOWCONSTRAINTS
            showConstraints  = opttf(pval,okargs{k},mfilename);
        case {8,9} % SHOWALIGNMENT
            if islogical(pval)
                if all(pval)
                    pval = 2;
                else
                    pval = 0;
                end
            end
            if ischar(pval)
                if any(strcmpi(pval,{'true','yes','on','t'}))
                    pval = 2;
                elseif any(strcmpi(pval,{'false','no','off','f'}))
                    pval = 0;
                end
            end
            if isscalar(pval) && pval==0
                showAlignment = 0;
            elseif isscalar(pval) && ~rem(pval,1) && pval>1 && pval<=nD
                showAlignment = pval;
            else
                if pval>nD
                    error('Bioinfo:samplealign:showAlignmentTooLarge',...
                        ['SHOWALIGNMENT must be an index to one of the columns of X and Y\n',...
                        'to plot; trying to use column %d, but inputs only have %d column(s).'],pval,nD)
                else
                    error('Bioinfo:samplealign:showAlignmentInvalid',...
                        'SHOWALIGNMENT must be an index to one of the columns of X and Y to plot.')
                end
            end
        case {10,11} % SHOWNETWORK
            showNetwork  = opttf(pval,okargs{k},mfilename);
        case 12 % QUANTILE
            if isempty(pval)
                computeQuantile = false;
            elseif isscalar(pval) && pval>=0 && pval<=1
                quantileValue = pval;
            else
                error('Bioinfo:samplealign:InvalidQuantile',...
                    'QUANTILE must be a scalar between 0 and 1, or an empty array.')
            end
        case 13 %LINKAXES (undocummented)
            axisLink  = opttf(pval,okargs{k},mfilename);
    end
end
%%% Set defaults that are dependent of input arguments %%%
if widthIsDefault
    if bandIsDefault
        widthX = 10;
        widthY = 10;
    else
        widthX = Inf;
        widthY = Inf;
    end
end

function [k, pval] = pvpair(pname, theVal, okargs,mfile)
% PVPAIR Helper function that looks for partial matches of parameter names
% in a list of inputs and returns the parameter/value pair and matching
% number.
%
% [K, PVAL] = PVPAIR(PNAME, THEVAL, OKARGS) given input string PNAME,
% and corresponding value, THEVAL, finds matching name in the OKARGS list.
% Returns K, the index of the match, and PVAL, the parameter value.

% Copyright 2007 The MathWorks, Inc.
% $Revision: 1.1.6.1 $   $Date: 2007/09/11 11:42:47 $

k = find(strncmpi(pname, okargs,numel(pname)));
if numel(k) == 1
    pval = theVal;
    return
end

if isempty(k)
    xcptn = MException(sprintf('Bioinfo:%s:UnknownParameterName',mfile),...
        'Unknown parameter name: %s.',pname);
    xcptn.throwAsCaller;

elseif length(k)>1
    xcptn = MException(sprintf('Bioinfo:%s:AmbiguousParameterName',mfile),...
        'Ambiguous parameter name: %s.',pname);
    xcptn.throwAsCaller;
end

function tf = opttf(pval,okarg,mfile)
%OPTTF determines whether input options are true or false

% Copyright 2003-2007 The MathWorks, Inc.
% $Revision: 1.3.4.3 $   $Date: 2007/09/11 11:42:46 $


if islogical(pval)
    tf = all(pval);
    return
end
if isnumeric(pval)
    tf = all(pval~=0);
    return
end
if ischar(pval)
    truevals = {'true','yes','on','t'};
    k = any(strcmpi(pval,truevals));
    if k
        tf = true;
        return
    end
    falsevals = {'false','no','off','f'};
    k = any(strcmpi(pval,falsevals));
    if k
        tf = false;
        return
    end
end
if nargin == 1
    % return empty if unknown value
    tf = logical([]);
else
    okarg(1) = upper(okarg(1));
    xcptn = MException(sprintf('Bioinfo:%s:%sOptionNotLogical',mfile,okarg),...
        '%s must be a logical value, true or false.',...
        upper(okarg));
    xcptn.throwAsCaller;
end

function path = graphpred2path(pred,D)
%GRAPHPRED2PATH converts predecessor indices to paths.
%  
% PATH = GRAPHPRED2PATH(PRED,D) traces back a path by following the
% predecessor list in PRED starting at destination node D. PRED is a row
% vector of predecessor node indices and D is a scalar. The value of the 
% root (or source) node in PRED must be 0. PATH is a row vector listing the
% nodes from the root (or source) to D. If a NaN is found when following 
% the predecessor nodes, PRED2PATH returns an empty path. 
%
% When PRED and D are both row vectors, PATH is a row cell array with every
% column containing the path to the destination for every element in D. 
% When PRED is a matrix and D is a scalar, PATH is a column cell array with
% every row containing the path for every row in PRED.
% When PRED is a matrix and D is a row vector, PATH is a matrix cell array
% with every row containing the paths for the respective row in PRED, and
% every column containing the paths to the respective destination in D.
%
% If D is omitted, the paths to all the destinations are calculated for
% every predecessor listed in PRED.
%
% Example:
%   % Find the nodes from the root to one leaf in a phylogenetic tree, for
%   % instance, the GLR_HUMAN protein
%   
%   tr = phytreeread('pf00002.tree')
%   view(tr)
%   [CM,labels,dist] = getmatrix(tr);
%   root_loc = size(CM,1)
%   glr_loc = strmatch('GLR',labels)
%   [T,PRED] = graphminspantree(CM,root_loc);
%   PATH = graphpred2path(PRED,glr_loc)
%   labels(PATH)
%
% See also: GRAPHALLSHORTESTPATHS, GRAPHCONNCOMP, GRAPHISDAG,
% GRAPHISOMORPHISM, GRAPHISSPANTREE, GRAPHMAXFLOW, GRAPHMINSPANTREE,
% GRAPHSHORTESTPATH, GRAPHTHEORYDEMO, GRAPHTOPOORDER, GRAPHTRAVERSE.  


%   Copyright 2006-2008 The MathWorks, Inc.
%   $Revision: 1.1.6.5 $  $Date: 2008/06/16 16:33:03 $

[numPreds,numNodes] = size(pred);
if nargin<2; D = 1:numNodes; end

% check input arguments
prednonan = ~isnan(pred);
if (any(pred(prednonan) < 0)) || (any(rem(pred(prednonan),1)))
    error('pred2path:InvalidIndices','Elements in PRED must be integers between 0 and the number of nodes (%d), or NaNs.',numNodes)
end
if (any(pred(prednonan) > numNodes)) 
    error('pred2path:InvalidIndicesTransposed','size(PRED) suggests there is (are) %d node(s), but there are elements in PRED larger, is PRED transposed?',numNodes)
end
if (any(D < 1)) || (any(D > numNodes)) || (any(rem(D,1)))
    error('pred2path:InvalidDestination','Elements in D must be integers between 1 and %d.',numNodes)
end

% initialize output
path = cell(numPreds,numel(D));
p = zeros(1,numNodes);

% trace back predecessors
for s = 1:numPreds
   for i = 1:numel(D)
       k = 1;
       n = D(i);
       p(k) = n;
       while (n~=0) && (k<=numNodes) && (~isnan(n))
           k = k+1;
           n = pred(s,n);
           p(k) = n;
       end
       if isnan(n)
           n = 0;
           path{s,i} = [];
       else
           path{s,i} = p(k-1:-1:1);
       end
       if n; break; end
   end
   if n; break; end
end
if n 
    error('pred2path:cycleDetected','PRED is invalid, potential cycle detected at node %d in row %d of PRED.',n,s)
end

if numel(path)==1; path = path{1}; end


function [dist,path,pred] = graphshortestpath(G,S,varargin)
%GRAPHSHORTESTPATH solves the shortest path problem in graph.
%
% [DIST,PATH,PRED] = GRAPHSHORTESTPATH(G,S) determines the single source
% shortest paths from node S to all other nodes in the graph G. Weights of
% the edges are all nonzero entries in the n-by-n adjacency matrix
% represented by the sparse matrix G. DIST are the n distances from source
% to every node (using Inf for non-reachable nodes and zero for the source
% node). The PATH contains the winning paths to every node, and PRED
% contains the predecessor nodes of the winning paths. 
% 
% [DIST,PATH,PRED] = GRAPHSHORTESTPATH(G,S,D) determines the single
% source-single destination shortest path from node S to node D.
% 
% GRAPHSHORTESTPATH(...,'METHOD',METHOD) selects the algorithm to use,
% options are:
%    'BFS'          - Breadth First Search, assumes all the weights are
%                     equal, edges are nonzero entries in the sparse matrix
%                     G. Time complexity is O(n+e).
%   ['Dijkstra']    - Assumes that weights of the edges are all positive
%                     values in the sparse matrix G. Time complexity is
%                     O(log(n)*e). 
%    'Bellman-Ford' - Assumes that weights of the edges are all nonzero
%                     entries in the sparse matrix G. Time complexity is
%                     O(n*e). 
%    'Acyclic'      - The input graph must be acyclic. Assumes that weights 
%                     of the edges are all nonzero entries in the sparse
%                     matrix G. Time complexity is O(n+e).
% 
% Note: n and e are number of nodes and edges respectively.
% 
% GRAPHSHORTESTPATH(...,'DIRECTED',false) indicates that the graph G is
% undirected, upper triangle of the sparse matrix is ignored. Default is
% true.
% 
% GRAPHSHORTESTPATH(...,'WEIGHTS',W) provides custom weights for the edges,
% useful to indicate zero valued weights. W is a column vector with one
% entry for every edge in G, traversed column-wise.
% 
% Examples:
%   % Create a directed graph with 6 nodes and 11 edges
%   W = [.41 .99 .51 .32 .15 .45 .38 .32 .36 .29 .21];
%   DG = sparse([6 1 2 2 3 4 4 5 5 6 1],[2 6 3 5 4 1 6 3 4 3 5],W)
%   h = view(biograph(DG,[],'ShowWeights','on'))
%   % Find shortest path from 1 to 6
%   [dist,path,pred] = graphshortestpath(DG,1,6)
%   % Mark the nodes and edges of the shortest path
%   set(h.Nodes(path),'Color',[1 0.4 0.4])
%   edges = getedgesbynodeid(h,get(h.Nodes(path),'ID'));
%   set(edges,'LineColor',[1 0 0])
%   set(edges,'LineWidth',1.5)
%
%   % Solving the previous problem for an undirected graph
%   UG = tril(DG + DG')
%   h = view(biograph(UG,[],'ShowArrows','off','ShowWeights','on'))
%   % Find the shortest path between node 1 and 6
%   [dist,path,pred] = graphshortestpath(UG,1,6,'directed',false)
%   % Mark the nodes and edges of the shortest path
%   set(h.Nodes(path),'Color',[1 0.4 0.4])
%   fowEdges = getedgesbynodeid(h,get(h.Nodes(path),'ID'));
%   revEdges = getedgesbynodeid(h,get(h.Nodes(fliplr(path)),'ID'));
%   edges = [fowEdges;revEdges];
%   set(edges,'LineColor',[1 0 0])
%   set(edges,'LineWidth',1.5)
%
% See also: GRAPHALLSHORTESTPATHS, GRAPHCONNCOMP, GRAPHISDAG,
% GRAPHISOMORPHISM, GRAPHISSPANTREE, GRAPHMAXFLOW, GRAPHMINSPANTREE,
% GRAPHPRED2PATH, GRAPHTHEORYDEMO, GRAPHTOPOORDER, GRAPHTRAVERSE.
%
% References: 
%  [1]	E.W. Dijkstra "A note on two problems in connexion with graphs"
%       Numerische Mathematik, 1:269-271, 1959. 
%  [2]	R. Bellman "On a Routing Problem" Quarterly of Applied Mathematics,
%       16(1):87-90, 1958. 

%   Copyright 2006-2008 The MathWorks, Inc.
%   $Revision: 1.1.6.6 $  $Date: 2008/06/16 16:33:04 $

algorithms = {'bfs','dijkstra','bellman-ford','acyclic'};
algorithmkeys = {'spb','dij','bel','spa'};
debug_level = 0;

% set defaults of optional input arguments
D = 1:length(G); % will return shortest path to all other nodes
W = []; % no custom weights
algorithm  = 2; % defaults to dijkstra
directed = true;

% find out signature of input arguments
if nargin>2 && isnumeric(varargin{1})
    D = varargin{1};
    varargin(1) = [];
end

% read in optional PV input arguments
nvarargin = numel(varargin);
if nvarargin
    if rem(nvarargin,2) == 1
        error('shortestpath:IncorrectNumberOfArguments',...
            'Incorrect number of arguments to %s.',mfilename);
    end
    okargs = {'method','directed','weights'};
    for j=1:2:nvarargin-1
        pname = varargin{j};
        pval = varargin{j+1};
        k = find(strncmpi(pname,okargs,numel(pname)));
        if isempty(k)
            error('shortestpath:UnknownParameterName',...
                'Unknown parameter name: %s.',pname);
        elseif length(k)>1
            error('shortestpath:AmbiguousParameterName',...
                'Ambiguous parameter name: %s.',pname);
        else
            switch(k)
                case 1 % 'method'
                    algorithm = strmatch(lower(pval),algorithms); 
                    if isempty(algorithm) 
                        error('shortestpath:NotValidMethod',...
                              'String "%s" is not not a valid algorithm.',pval)
                    elseif numel(algorithm)>1
                         error('shortestpath:AmbiguousMethod',...
                              'String "%s" is ambiguous.',pval)
                    end
                case 2 % 'directed'
                    directed = opttf(pval,okargs{k},mfilename);
                case 3 % 'weights'
                    W = pval(:);
            end
        end
    end
end

% call the mex implementation of the graph algorithms
if nargout>1
    if isempty(W)
        [dist,pred] = graphalgs(algorithmkeys{algorithm},debug_level,directed,G,S);
    else
        [dist,pred] = graphalgs(algorithmkeys{algorithm},debug_level,directed,G,S,W);
    end    
else
    if isempty(W)
        dist = graphalgs(algorithmkeys{algorithm},debug_level,directed,G,S);
    else
        dist = graphalgs(algorithmkeys{algorithm},debug_level,directed,G,S,W);
    end
end

dist = dist(D);

% calculate paths
if nargout>1
    path = graphpred2path(pred,D);
end