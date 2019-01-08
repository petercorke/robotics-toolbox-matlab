%SIMULINKEXT Return file extension of Simulink block diagrams.
%
% str = simulinkext() is either
% - '.mdl' if Simulink version number is less than  8
% - '.slx' if Simulink version numberis larger or equal to 8
%
% Notes::
% The file extension for Simulink block diagrams has changed from Matlab 2011b to Matlab 2012a. 
% This function is used for backwards compatibility.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also symexpr2slblock, doesblockexist, distributeblocks.

% Copyright (C) 2012-2013, by Joern Malzahn
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
%
% http://www.petercorke.com
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

function str = simulinkext()

if verLessThan('simulink','8')
    str = '.mdl';
else
    str = '.slx';
end