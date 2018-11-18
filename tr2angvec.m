%TR2ANGVEC Convert rotation matrix to angle-vector form
%
% [THETA,V] = TR2ANGVEC(R, OPTIONS) is rotation expressed in terms of an
% angle THETA (1x1) about the axis V (1x3) equivalent to the orthonormal rotation
% matrix R (3x3).
%
% [THETA,V] = TR2ANGVEC(T, OPTIONS) as above but uses the rotational part of the
% homogeneous transform T (4x4).
%
% If R (3x3xK) or T (4x4xK) represent a sequence then THETA (Kx1)is a vector 
% of angles for corresponding elements of the sequence and V (Kx3) are the 
% corresponding axes, one per row.
%
% Options::
% 'deg'   Return angle in degrees
%
% Notes::
% - For an identity rotation matrix both THETA and V are set to zero.
% - The rotation angle is always in the interval [0 pi], negative rotation
%   is handled by inverting the direction of the rotation axis.
% - If no output arguments are specified the result is displayed.
%
% See also ANGVEC2R, ANGVEC2TR, TRLOG.




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

function [theta_, n_] = tr2angvec(R, varargin)

    assert(ishomog(R) || isrot(R), 'RTB:tr2angvec:badarg', 'argument must be SO(3) or SE(3)');

    opt.deg = false;
    opt = tb_optparse(opt, varargin);
    
    % get the rotation submatrix(s)
    if ~isrot(R)
        R = t2r(R);
    end
    
    if size(R,3) > 1
        theta = zeros(size(R,3),1);
        v = zeros(size(R,3),3);
    end
    
    for i=1:size(R,3)  % for each rotation matrix in the sequence
        
        % There are a few ways to do this:
        %
        % 1.
        %
        % e = 0.5*vex(R - R');  % R-R' is skew symmetric
        % theta = asin(norm(e));
        % n = unit(e);
        %
        %  but this fails for rotations > pi/2
        %
        % 2.
        %
        % e = vex(logm(R));
        % theta = norm(e);
        % n = unit(e);
        %
        %  elegant, but 40x slower than using eig
        %
        % 3.
        %
        % Use eigenvectors, get angle from trace which is defined over -pi to
        % pi.  Don't use eigenvalues since they only give angles -pi/2 to pi/2.
        %
        % 4.
        %
        % Take the log of the rotation matrix
        
        Ri = R(:,:,i);
        
        % check the determinant
        assert( abs(det(Ri)-1) < 10*eps, 'RTB:tr2angvec:badarg', 'matrix is not orthonormal');
        
        [th,v] = trlog(Ri);
        theta(i) = th;
        n(i,:) = v;
        
        if opt.deg
            theta(i) = theta(i) * 180/pi;
            units = 'deg';
        else
            units = 'rad';
        end
        
        if nargout == 0
            % if no output arguments display the angle and vector
            fprintf('Rotation: %f %s x [%f %f %f]\n', theta(i), units, n(i,:));
        end
    end
    
    if nargout == 1
        theta_ = theta;
    elseif nargout == 2
        theta_ = theta;
        n_ = n;
    end