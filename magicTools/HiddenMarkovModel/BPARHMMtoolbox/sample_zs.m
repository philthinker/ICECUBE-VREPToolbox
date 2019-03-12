function [stateSeq INDS stateCounts] = sample_zs(data_struct,dist_struct,F,theta,obsModelType)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define and initialize parameters %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

numObj = length(data_struct);

Kz = size(dist_struct(1).pi_z,2);
Ks = size(dist_struct(1).pi_s,2);

% Initialize state count matrices:
N = zeros(Kz+1,Kz,numObj);
Ns = zeros(Kz,Ks,numObj);

% Preallocate INDS
for ii = 1:length(data_struct)
  T = length(data_struct(ii).blockSize);
  INDS(ii).obsIndzs(1:Kz,1:Ks) = struct('inds',sparse(1,T),'tot',0);
  stateSeq(ii) = struct('z',zeros(1,T),'s',zeros(1,data_struct(ii).blockEnd(end)));
end

for ii=1:length(data_struct)
    % Define parameters:
    [pi_z pi_init] = transformDistStruct(dist_struct(ii),F(ii,:));
    pi_s = dist_struct(ii).pi_s;
    
    T = length(data_struct(ii).blockSize);
    blockSize = data_struct(ii).blockSize;
    blockEnd = data_struct(ii).blockEnd;

    % Initialize state and sub-state sequences:
    z = zeros(1,T);
    s = zeros(1,sum(blockSize));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute likelihoods and messages %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Compute likelihood(kz,ks,u_i) of each observation u_i under each
    % parameter theta(kz,ks):
    Kz_inds = find(F(ii,:)>0);
    likelihood = compute_likelihood(data_struct(ii),theta,obsModelType,Kz_inds,Kz,Ks);

    % Compute backwards messages:
    [bwds_msg, partial_marg] = backwards_message_vec(likelihood, blockEnd, pi_z, pi_s);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Sample the state and sub-state sequences %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Sample (z(1),{s(1,1)...s(1,N1)}).  We first sample z(1) given the
    % observations u(1,1)...u(1,N1) having marginalized over the associated s's
    % and then sample s(1,1)...s(1,N1) given z(1) and the observations.

    totSeq = zeros(Kz,Ks);
    indSeq = zeros(T,Kz,Ks);

    for t=1:T
        % Sample z(t):
        if (t == 1)
            Pz = pi_init' .* partial_marg(:,1);
            obsInd = [1:blockEnd(1)];
        else
            Pz = pi_z(z(t-1),:)' .* partial_marg(:,t);
            obsInd = [blockEnd(t-1)+1:blockEnd(t)];
        end
        Pz   = cumsum(Pz);
        z(t) = 1 + sum(Pz(end)*rand(1) > Pz);

        % Add state to counts matrix:
        if (t > 1)
            N(z(t-1),z(t),ii) = N(z(t-1),z(t),ii) + 1;
        else
            N(Kz+1,z(t),ii) = N(Kz+1,z(t),ii) + 1;  % Store initial point in "root" restaurant Kz+1
        end

        % Sample s(t,1)...s(t,Nt) and store sufficient stats:
        for k=1:blockSize(t)
            % Sample s(t,k):
            if Ks > 1
                Ps = pi_s(z(t),:) .* likelihood(z(t),:,obsInd(k));
                Ps = cumsum(Ps);
                s(obsInd(k)) = 1 + sum(Ps(end)*rand(1) > Ps);
            else
                s(obsInd(k)) = 1;
            end

            % Add s(t,k) to count matrix and observation statistics:
            Ns(z(t),s(obsInd(k)),ii) = Ns(z(t),s(obsInd(k)),ii) + 1;
            totSeq(z(t),s(obsInd(k))) = totSeq(z(t),s(obsInd(k))) + 1;
            indSeq(totSeq(z(t),s(obsInd(k))),z(t),s(obsInd(k))) = obsInd(k);
        end
    end
    
    stateSeq(ii).z = z;
    stateSeq(ii).s = s;

    for jj = 1:Kz
        for kk = 1:Ks
            INDS(ii).obsIndzs(jj,kk).tot  = totSeq(jj,kk);
            INDS(ii).obsIndzs(jj,kk).inds = sparse(indSeq(:,jj,kk)');
        end
    end

end

stateCounts.N = N;
stateCounts.Ns = Ns;

return;
