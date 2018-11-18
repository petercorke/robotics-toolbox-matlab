%SerialLink.gencoords Vector of symbolic generalized coordinates
%
% Q = R.gencoords() is a vector (1xN) of symbols [q1 q2 ... qN].
%
% [Q,QD] = R.gencoords() as above but QD is a vector (1xN) of 
% symbols [qd1 qd2 ... qdN].
%
% [Q,QD,QDD] = R.gencoords() as above but QDD is a vector (1xN) of 
% symbols [qdd1 qdd2 ... qddN].
%
% See also SerialLink.genforces.



% Copyright (C) 1993-2017, by Peter I. Corke
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

function [q,qd,qdd] = gencoords(r)

    if nargout > 0
        for j=1:r.n
            q(j) = sym( sprintf('q%d', j), 'real' );
        end
    end

    if nargout > 1
        for j=1:r.n
            qd(j) = sym( sprintf('qd%d', j), 'real' );
        end
    end

    if nargout > 2
        for j=1:r.n
            qdd(j) = sym( sprintf('qdd%d', j), 'real' );
        end
    end
