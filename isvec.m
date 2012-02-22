%ISVEC Test if argument is a vector
%
% ISVEC(V) is true (1) if the argument V is a 3-vector, else false (0).
%
% ISVEC(V, L) is true (1) if the argument V is a vector of length L,
% either a row- or column-vector.  Otherwise false (0).
%
% Notes::
% - differs from MATLAB builtin function ISVECTOR, the latter returns true
%   for the case of a scalar, ISVEC does not.
%
% See also ISHOMOG, ISROT.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function h = isvec(v, l)
    if nargin == 1
            l = 3;
    end
    d = size(v);
    h = logical( length(d) == 2 && min(d) == 1 && numel(v) == l );

