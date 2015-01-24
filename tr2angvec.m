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
% - If no output arguments are specified the result is displayed.
%
% See also ANGVEC2R, ANGVEC2TR.



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

function [theta_, n_] = tr2angvec(R, varargin)

    opt.deg = false;
    
    opt = tb_optparse(opt, varargin);
    
    % get the rotation submatrix(s)
    if ~isrot(R)
        R = t2r(R);
    end
    
    % check the determinant
    if abs(det(R)-1) > 10*eps
           error('matrix not orthonormal rotation matrix');
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
        
        [v,d] = eig(R(:,:,i));

        unit_evec = abs(real(diag(d))-1) < 20*eps;
        
        switch sum(unit_evec)
            case 0
                % no unit eigenvalues, matrix is not orthonormal
                error('matrix not orthonormal rotation matrix');
                
            case 1
                % one unit eigenvalue, should always be this case
                k = find(unit_evec);
            otherwise
                % for the case of a matrix very close to unity, the results
                % become complex, with a conjugate pair of eigenvectors and one
                % with zero complex part.
                for k=1:3
                    if isreal(v(:,k))
                        break;
                    end
                end
        end
         
        
        % get the direction, eigenvector corresponding to real eigenvalue
        n(i,:) = real(v(:,k));

        % rotation comes from the trace
        ac = (trace(R(:,:,i)) - 1) / 2;
        ac = max( min(ac, 1), -1);  % clip it to robustly handle slight non-orthonormality
        theta(i) = acos( ac );
        
        if nargout == 0
            % if no output arguments display the angle and vector
            if opt.deg
                fprintf('Rotation: %f deg x [%f %f %f]\n', theta(i)*180/pi, n(i,1), n(i,2), n(i,3));
            else
                fprintf('Rotation: %f rad x [%f %f %f]\n', theta(i), n(i,1), n(i,2), n(i,3));
            end
        end
    end
    
    if ~isreal(theta) || ~isreal(n)
        error('complex');
    end
    
    if nargout == 1
        theta_ = theta;
    elseif nargout == 2
        theta_ = theta;
        n_ = n;
    end
