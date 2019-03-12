function [DM unique_true unique_est] = buildDistanceMatrix(true_labels,est_labels)

% map est to true

unique_true = unique(true_labels);
unique_est = unique(est_labels);

DM = zeros(length(unique_est),length(unique_true));

for ei = 1:length(unique_est)
    ind_ei = find(est_labels == unique_est(ei));
    for ti = 1:length(unique_true)
        num_incorrect = sum(true_labels(ind_ei)==unique_true(ti));
        DM(ei,ti) = num_incorrect;
    end
end

DM = 1-(DM./sum(sum(DM)));

        