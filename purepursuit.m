%PUREPURSUIT Find pure pursuit goal
%
% P = PUREPURSUIT(CP, R, PATH) is the current pursuit point (2x1) for a robot at
% location CP (2x1) following a PATH (Nx2).  The pursuit point is the
% closest point along the path that is a distance >= R from the current
% point CP.
%
% Reference::
% - A review of some pure-pursuit based tracking techniques for control of
%   autonomous vehicle, Samuel etal., Int. J. Computer Applications, Feb 2016
% - Steering Control of an Autonomous Ground Vehicle with Application to
%   the DARPA Urban Challenge, Stefan F. Campbell, Masters thesis, MIT, 2007.
%
% See also Navigation.

% Copyright (C) 1993-2019, by Peter I. Corke
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

function pstar = purepursuit(cp, R, traj)
    
    cp = cp(:)'; % ensure a row vector
    
    % find closest point on the path to current point
    d = colnorm((traj-cp)');  % rely on implicit expansion
    [~,i] = min(d);
    
    % find all points on the path at least R away
    k = find(d(i+1:end) >= R); % find all points beyond horizon
    if isempty(k)
        % no such points, we must be near the end, goal is the end
        pstar = traj(end,:);
    else
        % many such points, take the first one
        k = k(1);  % first point beyond horizon
        j = i+k-1; % index into traj array
        pstar = traj(j,:);
        %[j traj(j,:) norm(pstar-cp)]
    end
    
    
end