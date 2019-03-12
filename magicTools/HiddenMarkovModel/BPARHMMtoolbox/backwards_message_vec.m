
function [bwds_msg, partial_marg] = backwards_message_vec(likelihood,blockEnd,pi_z,pi_s)

% Allocate storage space
Kz = size(pi_z,2);
Ks = size(pi_s,2);
T  = length(blockEnd);

bwds_msg     = ones(Kz,T);
partial_marg = zeros(Kz,T);

% Compute marginalized likelihoods for all times, integrating s_t
if Kz==1 & Ks==1
    marg_like = squeeze(likelihood)';
else
    marg_like = squeeze(sum(likelihood .* pi_s(:,:,ones(1,1,blockEnd(end))),2));
end

% If necessary, combine likelihoods within blocks, avoiding underflow
if T < blockEnd(end)
  marg_like = log(marg_like+eps);

  block_like = zeros(Kz,T);
  block_like(:,1) = sum(marg_like(:,1:blockEnd(1)),2);
  for tt = 2:T
    block_like(:,tt) = sum(marg_like(:,blockEnd(tt-1)+1:blockEnd(tt)),2);
  end

  block_norm = max(block_like,[],1);
  block_like = exp(block_like - block_norm(ones(Kz,1),:));
else
  block_like = marg_like;
end

% Compute messages backwards in time
for tt = T-1:-1:1
  % Multiply likelihood by incoming message:
  partial_marg(:,tt+1) = block_like(:,tt+1) .* bwds_msg(:,tt+1);
  
  % Integrate out z_t:
  bwds_msg(:,tt) = pi_z * partial_marg(:,tt+1);
  bwds_msg(:,tt) = bwds_msg(:,tt) / sum(bwds_msg(:,tt));
end

% Compute marginal for first time point
partial_marg(:,1) = block_like(:,1) .* bwds_msg(:,1);

