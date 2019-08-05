classdef SimpleMarker
    %SimpleMarker A class for optical reflective markers data saving
    %   
    %   Haopeng Hu
    %   2019.06.24
    
    properties (Access = public)
        XYZ;    % positions of the marker
        N;      % number of the positions
        Time;   % time series of the marker (default:(0:1))
    end
    
    methods 
        function obj = SimpleMarker(XYZ,Time)
            % XYZ: N x 3, Time: N x 1, Time is not necessary
            if nargin < 2
                obj.Time = linspace(0,1,size(XYZ,1))';
            else
                obj.Time = Time;
            end
            obj.N = length(obj.Time);
            obj.XYZ = XYZ(1:obj.N,1:3);
        end
    end
    
    methods (Access = public)
        % Preliminary data processing functions
        obj = AverageFilter(obj,Interval);
    end
    
    methods (Access = public)
        % Plot functions
        [] = XYZScatter(obj,S,C);
    end
    
end

