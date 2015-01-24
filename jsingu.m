%JSINGU Show the linearly dependent joints in a Jacobian matrix
%
% JSINGU(J) displays the linear dependency of joints in a Jacobian matrix.
% This dependency indicates joint axes that are aligned and causes singularity.
%
% See also SerialLink.jacobn.


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

function jsingu(J)

    % convert to row-echelon form
    [R, jb] = rref(J);

    depcols = setdiff( 1:numcols(J), jb);

    fprintf('%d linearly dependent joints:\n', length(depcols));
    for d=depcols
        fprintf('  q%d depends on: ', d)
        for k=find(R(:,d))
            fprintf('q%d ', k);
        end
        fprintf('\n');
    end
