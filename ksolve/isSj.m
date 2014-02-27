function [b,sgn] = isSj(eq, j)
    %isSj Test if expression is sin(q_j)
%
% works for an array of equations
    
    Sj = mvar('sin(q%d)', j);
    
    for i=1:numel(eq)
        
        if isequaln(eq(i), Sj);
            b(i) = true;
            sgn(i) = 1;
        elseif isequaln(eq(i), -Sj);
            b(i) = true;
            sgn(i) = -1;
        else
            b(i) = false;
            sgn(i) = 0;
        end
    end
end
