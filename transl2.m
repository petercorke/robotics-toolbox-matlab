%TRANSL2 Create or unpack an SE(2) translational homogeneous transform
%
% Create a translational SE(2) matrix::
%
% T = TRANSL2(X, Y) is an SE(2) homogeneous transform (3x3) representing a
% pure translation.
%
% T = TRANSL2(P) is a homogeneous transform representing a translation or
% point P=[X,Y]. If P (Mx2) it represents a sequence and T (3x3xM) is a
% sequence of homogenous transforms such that T(:,:,i) corresponds to the
% i'th row of P.
%
% Extract the translational part of an SE(2) matrix::
%
% P = TRANSL2(T) is the translational part of a homogeneous transform as a
% 2-element column vector.  If T (3x3xM) is a homogeneous transform
% sequence the rows of P (Mx2) are the translational component of the
% corresponding transform in the sequence.
%
% Notes::
% - Somewhat unusually this function performs a function and its inverse.  An
%   historical anomaly.
%
% See also SE2.t, ROT2, ISHOMOG2, TRPLOT2, TRANSL.


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

function T = transl2(x, y)
    if nargin == 1
        if ishomog2(x)
            if ndims(x) == 3
                % transl(T)  -> P, trajectory case
                T = squeeze(x(1:2,3,:))';
            else
                % transl(T)  -> P
                T = x(1:2,3);
            end
        elseif all(size(x) == [3 3])
            T = x(1:2,3);
        elseif length(x) == 2
            % transl(P) -> T
            t = x(:);
            T =    [eye(2)          t(:);
                0   0   1];
        else
            % transl(P) -> T, trajectory case
            n = size(x,1);
            T = repmat(eye(3,3), [1 1 n]);
            T(1:2,3,:) = x';
        end    
    elseif nargin == 2
        % transl(x,y) -> T
        t = [x; y];
        T =    [ eye(2) t; 0 0 1];        
    end
end

% one less function to upload for Cody/LTI assessments

function h = ishomog2(tr, rtest)
    d = size(tr);
    if ndims(tr) >= 2
        h =  all(d(1:2) == [3 3]);

        if h && nargin > 1
            h = abs(det(tr(1:2,1:2)) - 1) < eps;
        end
    else
        h = false;
    end
end