%TR2ANGVEC Convert rotation matrix to angle-vector form
%
% [THETA,V] = TR2ANGVEC(R) converts an orthonormal rotation matrix R into a 
% rotation of THETA (1x1) about the axis V (1x3).
%
% [THETA,V] = TR2ANGVEC(T) as above but uses the rotational part of the
% homogeneous transform T.
%
% If R (3x3xK) or T (4x4xK) represent a sequence then THETA (Kx1)is a vector 
% of angles for corresponding elements of the sequence and V (Kx3) are the 
% corresponding axes, one per row.
%
% Notes::
% - If no output arguments are specified the result is displayed.
%
% See also ANGVEC2R, ANGVEC2TR.


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

function [theta_, v_] = tr2angvec(R)

    if ~isrot(R)
        R = t2r(R);
    end

    if size(R,3) > 1
        theta = zeros(size(R,3),1);
        v = zeros(size(R,3),3);
        for i=1:size(R,3)
            [t,a] = tr2angvec(R(:,:,i));
            theta(i) = t;
            v(i,:) = a;
        end
        return
    end
    
    e = 0.5*vex(R - R');  % this is skew symmetric
    ne = norm(e);
    
    if ne < eps
        theta = 0;
        v = [0 0 0];
    else
        theta = asin( norm(e) ); % the norm gives the angle
        v = unit(e);  % the vector gives the length
    end
    


    if nargout == 0
        % if no output arguments display the angle and vector
        fprintf('Rotation: %f rad x [%f %f %f]\n', theta, v(1), v(2), v(3));
    elseif nargout == 1
        theta_ = theta;
    elseif nargout == 2
        theta_ = theta;
        v_ = v;
    end
