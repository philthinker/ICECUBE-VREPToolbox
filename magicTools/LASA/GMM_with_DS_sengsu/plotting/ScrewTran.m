%% Compute a translation along a screw axis
%% input: d is a vector 3x1, specifiying translation of an origin
%%        s is a unit vector 3x1, specifiying a direction of a screw
%% output: scalar, amount of translation along a screw

function res = ScrewTran(d, s)
    
    d_p = d - dot(s, d) .* s;
    res = sign(dot(s, d)) * dot(d - d_p, d - d_p) ^ 0.5;