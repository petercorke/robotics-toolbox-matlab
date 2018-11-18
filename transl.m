%TRANSL Create or unpack an SE(3) translational homogeneous transform
%
% Create a translational SE(3) matrix::
%
% T = TRANSL(X, Y, Z) is an SE(3) homogeneous transform (4x4) representing
% a pure translation of X, Y and Z.
%
% T = TRANSL(P) is an SE(3) homogeneous transform (4x4) representing a
% translation of P=[X,Y,Z]. If P (Mx3) it represents a sequence and T
% (4x4xM) is a sequence of homogeneous transforms such that T(:,:,i)
% corresponds to the i'th row of P.
%
% Extract the translational part of an SE(3) matrix::
%
% P = TRANSL(T) is the translational part of a homogeneous transform T as a
% 3-element column vector.  If T (4x4xM) is a homogeneous transform
% sequence the rows of P (Mx3) are the translational component of the
% corresponding transform in the sequence.
%
% [X,Y,Z] = TRANSL(T) is the translational part of a homogeneous transform
% T as three components.  If T (4x4xM) is a homogeneous transform sequence
% then X,Y,Z (1xM) are the translational components of the corresponding
% transform in the sequence.
%
% Notes::
% - Somewhat unusually this function performs a function and its inverse.  An
%   historical anomaly.
%
% See also SE3.t, SE3.transl.


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

function [t1,t2,t3] = transl(x, y, z)
    if nargin == 1
        if ishomog(x)
            if ndims(x) == 3
                % transl(T)  -> P, trajectory case
                if nargout == 1 
                    t1 = squeeze(x(1:3,4,:))';
                elseif nargout == 3
                    t1 = squeeze(x(1,4,:))';
                    t2 = squeeze(x(2,4,:))';
                    t3 = squeeze(x(3,4,:))';
                end
            else
                % transl(T)  -> P
                if nargout == 1 || nargout == 0
                    t1 = x(1:3,4);
                elseif nargout == 3
                    t1 = x(1,4);
                    t2 = x(2,4);
                    t3 = x(3,4);
                end
                    
            end
        elseif length(x) == 3
            % transl(P) -> T
            t = x(:);
            t1 =    [eye(3)          t(:);
                0   0   0   1];
        else
            % transl(P) -> T, trajectory case
            n = size(x,1);
            t1 = repmat(eye(4,4), [1 1 n]);
            t1(1:3,4,:) = x';
        end    
    elseif nargin == 3
        % transl(x,y,z) -> T
        t = [x; y; z];
        t1 =    [ eye(3) t; 0 0 0 1];
    else
        error('RTB:transl:badargs', 'should be 1 or 3 arguments');
    end
end

% one less function to upload for Cody/LTI assessments
function h = ishomog(tr, rtest)
    d = size(tr);
    if ndims(tr) >= 2
        h =  all(d(1:2) == [4 4]);

        if h && nargin > 1
            h = abs(det(tr(1:3,1:3)) - 1) < eps;
        end

    else
        h = false;
    end
end