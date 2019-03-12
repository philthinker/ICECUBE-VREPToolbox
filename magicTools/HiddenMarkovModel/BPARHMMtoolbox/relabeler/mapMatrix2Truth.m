function [relabeled_est_mat assignment] = mapMatrix2Truth(true_mat,est_mat)

DM = buildDistanceMatrix2(true_mat,est_mat);

[assignment, cost] = assignmentoptimal(DM);

new_label_count=size(true_mat,2)+1;

relabeled_est_mat = zeros(size(est_mat));
for ii=1:length(assignment)
    if assignment(ii)==0
        relabeled_est_mat(:,new_label_count)=est_mat(:,ii);
        assignment(ii) = new_label_count;
        new_label_count = new_label_count + 1;
    else
        relabeled_est_mat(:,assignment(ii))=est_mat(:,ii);
    end
end

%hamming_dist = sum(relabeled_est_labels~=true_labels)/length(true_labels);