function [ y ] = phi0123( x, T )
%PHI0123 a function used in value_update
% T: the temperature parameter

for i=1:length(x)
    if x>=0
        y=exp(-1*x/T);
    else
        y=-1*x/T+1;
    end
end

end

