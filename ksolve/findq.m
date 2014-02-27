%FINDQ Find the joint variables in expression
%
% Q = FINDQ(E) returns a list of integers indicating the joint variables found
% in the expression E.  For instance an instance of 'q1' would cause a 1 to be 
% returned and so on.
%
% Eg. findq('sin(q1)*cos(q2)+S3') -> [1 2]

function q = findq(s)
    
    q = [];
    
    for var=symvar(s)
        if isempty(var)
            break
        end
        varname = char(var);
        if varname(1) == 'q'
            q = [q str2num(varname(2:end))];
        end
    end
end
