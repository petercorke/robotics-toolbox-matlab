%TRCHAIN2 Chain 2D transforms from string
%
% T = TRCHAIN2(S, Q) is a homogeneous transform (3x3) that results from
% compounding a number of elementary transformations defined by the string
% S.  The string S comprises a number of tokens of the form X(ARG) where
% X is one of Tx, Ty or R.  ARG is the name of a variable in
% main workspace or qJ where J is an integer in the range 1 to N that
% selects the variable from the Jth column of the vector Q (1xN).
%
% For example:
%        trchain('R(q1)Tx(a1)R(q2)Ty(a3)R(q3)', [1 2 3])
%
% is equivalent to computing:
%        trot2(1) * transl2(a1,0) * trot2(2) * transl2(0,a3) * trot2(3)
%
% Notes::
% - The string can contain spaces between elements or on either side of ARG.
%
% See also trchain, trot2, transl2.


function T = trchain2(s, q)
    
    %s = 'R(q1)Tx(a1)R(q2)Tx(a3)R(q3)Tx(a3)';
    
    tokens = regexp(s, '\s*(?<op>R.?|T.)\(\s*(?<arg>[A-Za-z][A-Za-z0-9]*)\s*\)\s*', 'names');

    
    T = eye(3,3);
    joint = 1;
    
    for token = tokens

        % get the argument for this transform element
        if token.arg(1) == 'q'
            % from the passed in vector q
            try
                arg = q(joint);
            catch
                error('RTB:trchain2:badarg', 'vector q has insufficient values');
            end
            joint = joint+1;
        else
            % or the workspace
            try
                arg = evalin('base', token.arg);
            catch
                error('RTB:trchain2:badarg', 'variable %s does not exist', token.arg);
            end
        end
        
        % now evaluate the element and update the transform chain
        switch token.op
            case 'R'
                T = T * trot2(arg);
            case 'Tx'
                T = T * transl2(arg, 0);
            case 'Ty'
                T = T * transl2(0, arg);
            otherwise
                error('RTB:trchain2:badarg', 'unknown operator %s', token.op);
        end
    end
    