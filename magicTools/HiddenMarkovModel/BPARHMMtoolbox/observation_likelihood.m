function [total_log_likelihood neglog_c] = observation_likelihood(F,data_struct,obsModelType,dist_struct,theta)

pi_init = dist_struct.pi_init;
pi_z = dist_struct.pi_z;
pi_s = dist_struct.pi_s;
Kz = size(pi_z,2);
Ks = size(pi_s,2);

blockEnd = data_struct.blockEnd;

% Compute likelihood matrix:
Kz_inds = find(F>0);
[likelihood log_normalizer] = compute_likelihood(data_struct,theta,obsModelType,Kz_inds,Kz,Ks);

likelihood_Kz_inds = likelihood(Kz_inds,:,:);
[pi_z pi_init pi_s] = transformDistStruct(dist_struct,Kz_inds);

% Pass messages forward to integrate over the mode/state sequence:
[fwd_msg neglog_c] = forward_message_vec(likelihood_Kz_inds,log_normalizer,blockEnd,pi_z,pi_s,pi_init);

total_log_likelihood = sum(neglog_c);

return;
