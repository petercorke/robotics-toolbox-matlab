%SOLVE_JOINT Solve a trigonometric equation
%
% S = SOLVE_JOINT(EQ, J) solves the equation EQ=0 for the joint variable qJ.
% The result is a cell array of solutions.
%
% The equations must be of the form:
%  A cos(qJ) + B sin(qJ) = 0
%  A cos(qJ) + B sin(qJ) = C
%
% where A, B, C are arbitrarily complex expressions.  qJ can be the only
% joint variable in the expression.

function s = solve_joint(eq, j)
    % solve an equation for q_j which is implicitly equal to zero
        
    sinj = mvar('sin(q%d)', j);
    cosj = mvar('cos(q%d)', j);

    A = getcoef(eq, cosj);
    B = getcoef(eq, sinj);
    
    if isempty(A) || isempty(B)
        warning('don''t know how to solve this kind of equation');
    end

    C = -simple(eq - A*cosj - B*sinj);
    
    if C == 0
        % A cos(q) + B sin(q) = 0
        s(2) = atan2(A, -B);
        s(1) = atan2(-A, B);
    else
        % A cos(q) + B sin(q) = C
        r = sqrt(A^2 + B^2 - C^2);
        phi = atan2(A, B);
        
        s(2) = atan2(C, r) - phi;
        s(1) = atan2(C, -r) - phi;
    end
if nargout == 0
    try
    eval(s)
    catch
        s
    end
end
end

