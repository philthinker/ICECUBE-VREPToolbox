function [ p_tform ] = axang_trvec2tform( p_axang, p_trvec )
% Transform axang+trvec to tform
% p_axang: axang data
% p_trvec: trvec data
% p_tform: the output tform data
% Robotics System Toolbox is required

p_tform = axang2tform(p_axang);
for i = 1:3
    p_tform(i,4) = p_trvec(i);
end

end

