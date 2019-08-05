classdef Marker < SimpleMarker
    %Marker A class to deal with optical marker
    %   TXYZ: N x 4 or N x 3, the positions
    %   Time: N x 1, the time series
    
    %   Haopeng Hu
    %   2019.08.05
    %   All rights reserved
    
    properties
    end
    
    methods
        function obj = Marker(TXYZ,Time)
            %Marker You can build it by XYZ, TXYZ and XYZ + Time
            %   arg1: N x 4 or N x 3, the positions
            %   arg2: N x 1, the time series
            if nargin < 1
                % Init. an empty Marker
                TXYZ = [0,0,0,0];
            elseif nargin == 1
                % Default time series
                time = linspace(0,1,size(TXYZ,1));
            else
                time = Time;
            end
            if size(TXYZ,2) > 3
                % TXYZ is given
                time = TXYZ(:,1);
                xyz = TXYZ(:,2:4);
            else
                xyz = TXYZ;
            end
            obj@SimpleMarker(xyz,time);
        end
        
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

