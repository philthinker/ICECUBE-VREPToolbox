function [ alpha1 ] = step_update0122( alpha0,k,k_max,T )
%STEP_UPDATE0122 update the step of value updation
% alpha0: 1 x 6, the current step
% k: the current episode
% k_max: the maximum episode number of step updation
% T: the temperature parameter >0
% alpha2: 1 x 6, the new step

if k<= k_max
%     alpha1 = (k/T)*alpha0;
    alpha1 = exp(k/T)*alpha0;
else
    alpha1 = exp(k_max/T)*alpha0;
end

end

