function [pi_z pi_init] = transformDistStruct(dist_struct,feature_vec)

Kz = length(feature_vec);
pi_z = dist_struct.pi_z.*repmat(feature_vec,[Kz,1]);
pi_z = pi_z./repmat(sum(pi_z,2),[1,Kz]);
pi_init = dist_struct.pi_init.*feature_vec;
pi_init = pi_init./sum(pi_init);