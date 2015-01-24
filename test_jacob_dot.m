% test harness for jacob_dot

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

if exist('p560') == 0
    mdl_puma560
end

e = 1e-4;
col = 1;
dq=[0 0 0 0 0 0];
dq(col) = 1;

Jd = (p560.jacob0(qn+dq*e)-p560.jacob0(qn))/e;
%Jd(:,col)
Jd
jacob_dot(p560, qn, dq)

