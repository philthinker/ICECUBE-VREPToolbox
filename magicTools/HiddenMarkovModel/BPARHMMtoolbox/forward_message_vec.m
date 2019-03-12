function [fwd_msg, neglog_c, block_like] = forward_message_vec(likelihood,loglike_normalizer,blockEnd,pi_z,pi_s,pi_init)

% Allocate storage space
Kz = size(pi_z,2);
Ks = size(pi_s,2);
T  = length(blockEnd);

fwd_msg  = ones(Kz,T);
neglog_c = zeros(1,T);

% Compute marginalized likelihoods for all times, integrating s_t
%marg_like = squeeze(sum(likelihood .* pi_s(:,:,ones(1,1,blockEnd(end))),2));
marg_like = reshape(sum(likelihood .* pi_s(:,:,ones(1,1,blockEnd(end))),2), ...
  size(likelihood,1), size(likelihood,3));

% If necessary, combine likelihoods within blocks, avoiding underflow
if T < blockEnd(end)
  marg_like = log(marg_like+eps);

  block_like = zeros(Kz,T);
  block_like(:,1) = sum(marg_like(:,1:blockEnd(1)),2);
  % Initialize normalization constant to be that due to the likelihood:
  neglog_c(1) = sum(loglike_normalizer(1:blockEnd(1)));
  for tt = 2:T
    block_like(:,tt) = sum(marg_like(:,blockEnd(tt-1)+1:blockEnd(tt)),2);
    neglog_c(tt) = sum(loglike_normalizer(blockEnd(tt-1)+1:blockEnd(tt)));
  end

  block_norm = max(block_like,[],1);
  block_like = exp(block_like - block_norm(ones(Kz,1),:));
  % Add on the normalization constant used after marginalizing the s_t's:
  neglog_c = neglog_c + block_norm;
else
   
  block_like = marg_like;
  % If there is no blocking, the normalization is simply due to the
  % likelihood computation:
  neglog_c = squeeze(loglike_normalizer)';
end

% Compute marginal for first time point
fwd_msg(:,1) = block_like(:,1) .* pi_init';
sum_fwd_msg = sum(fwd_msg(:,1));
fwd_msg(:,1) = fwd_msg(:,1) / sum_fwd_msg;
% Add the constant from normalizing the forward message:
neglog_c(1) = neglog_c(1)+log(sum_fwd_msg);

% Compute messages forward in time
pi_zT = pi_z';

for tt = 1:T-1
  % Integrate out z_t and multiply by likelihood: 
  fwd_msg(:,tt+1) = (pi_zT * fwd_msg(:,tt)) .* block_like(:,tt+1);
  sum_fwd_msg = sum(fwd_msg(:,tt+1));
  fwd_msg(:,tt+1) = fwd_msg(:,tt+1) / sum_fwd_msg;
 
  % Add the constant from normalizing the forward message:
  neglog_c(tt+1) = neglog_c(tt+1)+log(sum_fwd_msg);
end