%TRANSL Create translational transform
%
% T = TRANSL(X, Y, Z) is a homogeneous transform representing a 
% pure translation.
%
% T = TRANSL(P) is a homogeneous transform representing a translation or 
% point P=[X,Y,Z]. If P (Mx3) it represents a sequence and T (4x4xM)
% is a sequence of homogenous transforms such that T(:,:,i) corresponds to
% the i'th row of P.
%
% P = TRANSL(T) is the translational part of a homogenous transform as a 
% 3-element column vector.  If T (4x4xM) is a homgoeneous transform sequence 
% the rows of P (Mx3) are the translational component of the corresponding 
% transform in the sequence.
%
% Notes::
% - Somewhat unusually this function performs a function and its inverse.  An
%   historical anomaly.
%
% See also CTRAJ.


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

function T = transl(x, y, z)
    if nargin == 1
        if ishomog(x)
            if ndims(x) == 3
                % transl(T)  -> P, trajectory case
                T = squeeze(x(1:3,4,:))';
            else
                % transl(T)  -> P
                T = x(1:3,4);
            end
        elseif all(size(x) == [3 3])
            T = x(1:2,3);
        elseif length(x) == 2
            % transl(P) -> T
            t = x(:);
            T =    [eye(2)          t(:);
                0   0   1];
        elseif length(x) == 3
            % transl(P) -> T
            t = x(:);
            T =    [eye(3)          t(:);
                0   0   0   1];
        else
            % transl(P) -> T, trajectory case
            n = numrows(x);
            T = repmat(eye(4,4), [1 1 n]);
            T(1:3,4,:) = x';
        end    
    elseif nargin == 2
        % transl(x,y) -> T
        t = [x; y];
        T =    rt2tr( eye(2), t);        
    elseif nargin == 3
        % transl(x,y,z) -> T
        t = [x; y; z];
        T =    rt2tr( eye(3), t);
    end
