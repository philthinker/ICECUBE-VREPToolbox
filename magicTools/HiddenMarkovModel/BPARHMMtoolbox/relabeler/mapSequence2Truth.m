function [relabeled_est_labels hamming_dist assignment relabeled_true_labels] = mapSequence2Truth(true_labels,est_labels)

true_labels = map2smallestIntegers(true_labels,max(true_labels));
%est_labels = map2smallestIntegers(est_labels,max(est_labels));

[DM unique_true unique_est] = buildDistanceMatrix(true_labels,est_labels);

if isempty(setdiff(0,unique_true))
    error('need to shift true_labels')
elseif isempty(setdiff(0,unique_est))
    error('need to shift est_labels')
end

[assignment, cost] = assignmentoptimal(DM);

new_label_count=max(unique_true)+1;

relabeled_est_labels = est_labels;
for ii=1:length(assignment)
    if assignment(ii)==0
        relabeled_est_labels(find(est_labels==unique_est(ii)))=new_label_count;
        new_label_count = new_label_count + 1;
        assignment(ii) = new_label_count;
    else
        relabeled_est_labels(find(est_labels==unique_est(ii)))=assignment(ii);
    end
end

hamming_dist = sum(relabeled_est_labels~=true_labels)/length(true_labels);

relabeled_true_labels = true_labels;