%TRCHAIN2 Chain 2D transforms from string
%
% T = TRCHAIN2(S, Q) is a homogeneous transform (3x3) that results from
% compounding a number of elementary transformations defined by the string
% S.  The string S comprises a number of tokens of the form X(ARG) where
% X is one of Tx, Ty or R.  ARG is the name of a variable in
% MATLAB workspace or qJ where J is an integer in the range 1 to N that
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
% - Works for symbolic variables in the workspace and/or passed in via the 
%   vector Q.
% - For symbolic operations that involve use of the value pi, make sure you
%   define it first in the workspace: pi = sym('pi');
%
% See also trchain, trot2, transl2.

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com


function T = trchain2(s, q)
    
    %s = 'R(q1)Tx(a1)R(q2)Tx(a3)R(q3)Tx(a3)';
    
    tokens = regexp(s, '\s*(?<op>R.?|T.)\(\s*(?<arg>[A-Za-z][A-Za-z0-9]*)\s*\)\s*', 'names');

    if isa(q, 'symfun')
        q = formula(q);
    end
    
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
    
    if isa(q, 'symfun')
        T = formula(T);
    end
    
