function [ outExp ] = ternaryOperator( logicExp, trueExp, falseExp )
%ternaryOperator Similar to the ternary operator in C language
% outExp = logicExp?trueExp:falseExp

if logicExp
    outExp = trueExp;
else
    outExp = falseExp;
end

end

