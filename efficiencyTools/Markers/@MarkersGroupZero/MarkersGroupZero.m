classdef MarkersGroupZero
    %MarkersGroupZero A class to deal with a group of optical markers
    %   markers: 1 x M or M x 1 Marker
    %   The MATLAB Robotics System Toolbox and Peter Corke's Robotics
    %   Toolbox is recommended. It is assumed that the movements of the
    %   Markers are recorded simultaneously, i.e. the Markers share the
    %   same time series.
    
    %   Haopeng Hu
    %   2019.08.05
    %   All rights reserved
    
    properties (Access = public)
        Markers;    % The array of Marker
        Positions;  % The positions of the markergroup
        SO3;        % The rotation matrices of the markergroup
        SE3;        % The homogeneous matrices of the markergroup
    end
    
    properties (Access = protected)
        M;          % The num. of Markers
        N;          % The num. of Markers' frame
        Time;       % The time series of the Markers
    end
    
    methods
        function obj = MarkersGroupZero(markers)
            %MarkersGroupZero Init. the markersgroup by an array of markers
            %   markers: 1 x M or M x 1 Marker
            obj.M = length(markers);
            obj.Markers = markers;
            obj.Time = markers(1).Time;
            obj.N = length(obj.Time);
            obj.Positions = obj.centerCompute();
            if obj.M >= 3
                % There are at least 3 Markers
                obj.SO3 = obj.rotCompute();
                obj.SE3 = obj.tformCompute();
            else
                obj.SO3 = zeros(obj.N,1);
                obj.SE3 = zeros(obj.N,1);
            end
        end
    end
    
    methods (Access = public)
        % Figure
        function [] = plot3Markers(obj)
            %plot3Markers Plot the path of all the Markers
            %   The starting point is BLUE while the terminal point is RED
            labelMarkers = cell(1,obj.M);
            figure;
            for i = 1:obj.M
                labelMarkers{i} = strcat('Marker ',int2str(i));
                plot3(obj.Markers(i).XYZ(:,1),obj.Markers(i).XYZ(:,2),obj.Markers(i).XYZ(:,3));
                hold on;
            end
            legend(labelMarkers);
%             for i = 1:obj.M
%                 obj.plotXYCircle(obj.Markers(i).XYZ(1,1:3),10,'b');
%                 obj.plotXYCircle(obj.Markers(i).XYZ(end,1:3),10,'r');
%             end
            xlabel('x'); ylabel('y'); zlabel('z');
            grid on; axis equal;
        end
        function [] = plot3MarkersGroup(obj)
            %plot3MarkersGroup Plot the path of the center
            %   The starting point is BLUE while the terminal point is RED
            figure;
            plot3(obj.Positions(:,1),obj.Positions(:,2),obj.Positions(:,3));
            hold on;
%             obj.plotXYCircle([obj.Positions(1,1),obj.Positions(1,2),obj.Positions(1,3)],10,'b');
%             obj.plotXYCircle([obj.Positions(end,1),obj.Positions(end,2),obj.Positions(end,3)],10,'r');
            xlabel('x'); ylabel('y'); zlabel('z');
            grid on; axis equal;
        end
    end
    
    methods (Access = protected)
        XYZ = centerCompute(obj);
        [] = plotXYCircle(obj,center,radius,color);
        [SO3,p] = threeMarkerPos(obj,queryMarkers,originMarkers);
        SO3 = rotCompute(obj);
        SE3 = tformCompute(obj);
    end
end

