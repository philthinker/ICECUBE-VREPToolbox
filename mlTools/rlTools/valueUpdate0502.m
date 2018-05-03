function [ updatedValues ] = valueUpdate0502( cmd,R,points,selectedIndices, testedIndices, alpha )
%valueUpdate0502 Value updation strategy
% cmd: User's command
% R: Reward
% points: the values to be updated
% selectedIndices: Indices selected
% testedIndices: Indices unselected
% alpha: update step
% updatedValues: the updated values


updatedValues = points;
if cmd == 1
    % Accepted trajectory
    if R > 0
        updatedValues(selectedIndices) = updatedValues(selectedIndices) + alpha(1)*R;
        updatedValues(testedIndices) = updatedValues(testedIndices) - alpha(2)*R*10;
    else
        updatedValues(selectedIndices) = updatedValues(selectedIndices) + alpha(1)*R*0.1;
        updatedValues(testedIndices) = updatedValues(testedIndices) + alpha(2)*R*0.2;
    end
elseif cmd == 0
    % Unaccepted trajectory
    if R > 0
        updatedValues(selectedIndices) = updatedValues(selectedIndices) + alpha(3)*R*0.1;
        updatedValues(testedIndices) = updatedValues(testedIndices)+alpha(4)*phi0123(R,1.0)*0.2;
    else
        updatedValues(selectedIndices) = updatedValues(selectedIndices) + alpha(3)*R;
        updatedValues(testedIndices) = updatedValues(testedIndices)+alpha(4)*phi0123(R,1.0);
    end
end

end

