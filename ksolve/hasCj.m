%HASCJ Check if expression contains cosine term
%
% S=HASCJ(E, I) is true if the expression contains 'cos(qI)'

function b = hasCj(eq, j)
    %hasCj Test if expression contains cos(q_j)
%
% works for an array of equations
    
    Cj = sym( sprintf('cos(q%d)', j) );
    
    for i=1:numel(eq)
        
        e = children( collect(eq(i), Cj) );
        e = children(e(1));
        b(i) = numel(e) == 2 && isequaln(e(2), Cj);
    end
end
