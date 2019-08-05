function [obj] = AverageFilter(obj,Interval)
%AverageFilter data filter by averaging over an interval
%   Interval: Integer, the interval
%   @SimpleMarker

%% Data init.

N = obj.N;
XYZ = obj.XYZ;
Time = obj.Time;

%% Data processing

ITVAL = floor(Interval(1,1));
if N > ITVAL
    num = floor(N/ITVAL);
    n = 1;
    xyz = zeros(num,3); time = zeros(num,1);
    for i = 1:ITVAL:N
        xyz(n,:) = mean(XYZ(i:i+ITVAL-1,:),1);
        time(n) = Time(i);
        n = n + 1;
        if n >= num
            xyz(n,:) = mean(XYZ(i+ITVAL:N,:),1);
            time(n) = Time(i+ITVAL);
            break;
        end
    end
end
XYZ = xyz;
Time = time;

%% Object update

obj = SimpleMarker(XYZ,Time);

end

