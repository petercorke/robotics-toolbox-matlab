%TRINTERP2 Interpolate SE(2) homogeneous transformations
%
% T = TRINTERP2(T0, T1, S) is a homogeneous transform (3x3) interpolated
% between T0 when S=0 and T1 when S=1.  T0 and T1 are both homogeneous
% transforms (3x3).  If S (Nx1) then T (3x3xN) is a sequence of
% homogeneous transforms corresponding to the interpolation values in S.
%
% T = TRINTERP2(T1, S) as above but interpolated between the identity matrix
% when S=0 to T1 when S=1.
%
% See also trinterp, SE3.interp, UnitQuaternion.




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

function T = trinterp2(A, B, C)
    
    if nargin == 3
        %	TR = TRINTERP(T0, T1, r)
        T0 = A; T1 = B; r = C;
        
        if length(r) == 1 && r > 1 && (r == floor(r))
            % integer value
            r = [0:(r-1)] / (r-1);
        end
        assert(all(r>=0 & r<=1), 'RTB:trinterp2:badarg', 'values of S outside interval [0,1]');
        
        th0 = atan2(T0(2,1), T0(1,1));
        th1 = atan2(T1(2,1), T1(1,1));
        
        p0 = transl2(T0);
        p1 =transl2(T1);
        
        for i=1:length(r)
            th = th0*(1-r(i)) + r(i)*th1;
            pr = p0*(1-r(i)) + r(i)*p1;
            
            T(:,:,i) = rt2tr(rot2(th), pr);
        end
    elseif nargin == 2
        %	TR = TRINTERP(T, r)
        T1 = A; r = B;
        
        if length(r) == 1 && r > 1 && (r == floor(r))
            % integer value
            r = [0:(r-1)] / (r-1);
        elseif any(r<0 | r>1)
            error('RTB:trinterp2:badarg', 'values of S outside interval [0,1]');
        end
        
        th1 = atan2(T1(2,1), T1(1,1));
        p1 = transl2(T1);
        
        for i=1:length(r)
            th = r(i)*th1;
            pr = r*p1;
            
            T(:,:,i) = rt2tr(rot2(th), pr);
        end

    else
        error('RTB:trinterp2:badarg', 'must be 2 or 3 arguments');
    end    
