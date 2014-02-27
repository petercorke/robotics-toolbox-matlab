%HASSJ Check if expression contains sine term
%
% S=HASsJ(E, I) is true if the expression contains 'sin(qI)'
s
function b = hasSj(eq, j)
    %hasSj Test if expression contains sin(q_j)
%
% works for an array of equations
    
    Sj = sym( sprintf('sin(q%d)', j) );
    
    for i=1:numel(eq)
        
        e = children( collect(eq(i), Sj) );
        e = children(e(1));
        b(i) = numel(e) == 2 && isequaln(e(2), Sj);
    end
end
