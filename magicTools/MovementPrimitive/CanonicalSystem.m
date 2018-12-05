function [ dot_z ] = CanonicalSystem( t, z, dmp )
%CanonicalSystem Dynamics of the canonical system
%   dot_z = -tau*alpha_z*z

dot_z = -1 * dmp.tau * dmp.alpha_z * z;

end

