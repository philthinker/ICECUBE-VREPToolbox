function [F_sorted sort_ind] = lof(F)

Kz = size(F,2);
val = zeros(1,Kz);

for kk=1:Kz
    col_kk = num2str(F(:,kk)');
    val(kk) = bin2dec(col_kk);
end

[val_sort sort_ind] = sort(val,'descend');
F_sorted = F(:,sort_ind);

return;