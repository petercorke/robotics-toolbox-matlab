%TROTY Rotation about Y axis
%
% T = TROTY(THETA) is a homogeneous transformation (4x4) representing a rotation 
% radians about the y-axis.
%
% T = TROTY(THETA, 'deg') as above but THETA is in degrees.
%
% Notes::
% - Translational component is zero.
%
% See also ROTY, TROTX, TROTZ.


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

function T = troty(t, varargin)
	T =    [roty(t, varargin{:}) [0 0 0]'; 0 0 0 1];
