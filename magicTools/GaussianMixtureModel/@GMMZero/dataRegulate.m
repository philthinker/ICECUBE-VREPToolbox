function [Data] = dataRegulate(obj,Demos)
%dataRegulate Regulate the demos data into one matrix
%   Demos: 1 x D cells, we assume that the vectors in each cell share the
%   same num. of columns equaling to obj.nVar.
%   @GMMZero

nSample = length(Demos);
nData = zeros(nSample,1);
for i = 1:nSample
    nData(i) = size(Demos{i},1);
end
Data = zeros(sum(nData),obj.nVar);
nTemp = 0;
for i = 1:nSample
    Data(nTemp+1:nTemp+nData(i),:) = Demos{i};
    nTemp = nTemp + nData(i);
end

end

