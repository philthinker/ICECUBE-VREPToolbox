function DM = buildDistanceMatrix2(true_mat,est_mat)

DM = zeros(size(est_mat,2),size(true_mat,2));

for ei = 1:size(est_mat,2)
    for ti = 1:size(true_mat,2);
        num_incorrect = sum(true_mat(:,ti)==est_mat(:,ei));
        DM(ei,ti) = num_incorrect;
    end
end

DM = 1-(DM./sum(sum(DM)));

        