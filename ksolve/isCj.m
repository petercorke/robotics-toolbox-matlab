function [b,sgn] = isCj(eq, j)
    %isCj Test if expression is cos(q_j)
%
% works for an array of equations
    
    Cj = mvar('cos(q%d)', j);
    
    for i=1:numel(eq)
        
        if isequaln(eq(i), Cj);
            b(i) = true;
            sgn(i) = 1;
        elseif isequaln(eq(i), -Cj);
            b(i) = true;
            sgn(i) = -1;
        else
            b(i) = false;
            sgn(i) = 0;
        end
    end
end
