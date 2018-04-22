function [ p_tform ] = eul_trvec2tform( p_eul, p_trvec )
%EUL_TRVEC2TFORM Transform eul together with trvec to tform
% p_rotm: eul
% p_trvec: trvec


p_tform = eul2tform(p_eul);

p_tform(1,4) = p_trvec(1);
p_tform(2,4) = p_trvec(2);
p_tform(3,4) = p_trvec(3);

end


