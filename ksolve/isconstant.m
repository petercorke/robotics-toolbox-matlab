%ISCONSTANT Determine if an expression is free of joint variables
%
% S = ISCONSTANT(E) is true if the expression E contains no joint variables such
% q1, q2 etc.

function s = isconstant(eq)
    
    s = isempty(findq(eq));
