%HASONLY Determine if an expression contains only certain joint variables
%
% S = HASONLY(E L) is true if the joint variables (q1, q2 etc.) in the expression E 
% are listed in the vector L.
%
% Eg. hasonly('sin(q1)*cos(q2)*cos(q4)', [1 2 3]) -> true
% Eg. hasonly('sin(q1)*cos(q2)*cos(q4)', [1]) -> false

function s = hasonly(eq, j)
    
    q = findq(eq);
    if isempty(q)
        s = false;
    else
        s = all(ismember(j, findq(eq)));
    end