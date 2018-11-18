%TRLOG logarithm of SO(3) or SE(3) matrix
%
% S = trlog(R) is the matrix logarithm (3x3) of R (3x3)  which is a skew
% symmetric matrix corresponding to the vector theta*w where theta is the
% rotation angle and w (3x1) is a unit-vector indicating the rotation axis.
%
% [theta,w] = trlog(R) as above but returns directly theta the rotation
% angle and w (3x1) the unit-vector indicating the rotation axis.
%
% S = trlog(T) is the matrix logarithm (4x4) of T (4x4)  which has a (3x3)
% skew symmetric matrix upper left submatrix corresponding to the vector
% theta*w where theta is the rotation angle and w (3x1) is a unit-vector
% indicating the rotation axis, and a translation component.
%
% [theta,twist] = trlog(T) as above but returns directly theta the rotation
% angle and a twist vector (6x1) comprising [v w].
%
% Notes::
% - Efficient closed-form solution of the matrix logarithm for arguments that are
%   SO(3) or SE(3).
% - Special cases of rotation by odd multiples of pi are handled.
% - Angle is always in the interval [0,pi].
%
% References::
% - "Mechanics, planning and control"
%   Park & Lynch, Cambridge, 2016.
%
% See also trexp, trexp2, Twist.

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

function [o1,o2] = trlog(T)
    
    if isrot(T)
        % deal with rotation matrix
        
        % closed form solution for matrix logarithm of a homogeneous transformation (Park & Lynch)
        % that handles the special cases
        
        % for now assumes T0 is the world frame
        
        R = T;
        
        if abs(trace(R) - 3) < 100*eps
            % matrix is identity
            
            w = [0 0 0]';
            theta = 0;
            
        elseif abs(trace(R) + 1) < 100*eps
            % tr R = -1
            % rotation by +/- pi, +/- 3pi etc
            
            [mx,k] = max(diag(R));
            I = eye(3,3);
            col = R(:,k) + I(:,k);
            w = col / sqrt(2*(1+mx));
            
            theta = pi;
            
%             skw = logm(R);
%             w = vex( skw );
            
        else
            % general case
            theta = acos( (trace(R)-1)/2 );
            
            skw = (R-R')/2/sin(theta);
            w = vex(skw);   % is a unit vector
            
        end
        
        if nargout <= 1
            o1 = skew(w*theta);
        elseif nargout ==2
            o1 = theta;
            o2 = w;
        end

    elseif ishomog(T)
        % SE(3) matrix
        
        [R,t] = tr2rt(T);
        
        if abs(trace(R) - 3) < 100*eps
            % rotation matrix is identity, theta=0
            w = [0 0 0]';
            v = t;
            theta = 1;
            skw = zeros(3,3);
            
        else
            [theta, w] = trlog(R);
            skw = skew(w);
            
            Ginv = eye(3,3)/theta - skw/2 + ( 1/theta - cot(theta/2)/2 )*skw^2;
            v = Ginv * t;
        end
        
        if nargout <= 1
            o1 = [skw v; 0 0 0 0]*theta;
        elseif nargout ==2
            o1 = theta;
            o2 = [v; w];
        end
    else
        error('RTB:trlog:badarg', 'expect SO(3) or SE(3) matrix');
    end
        
end

%     [th,w] = tr2angvec(R);
%     w = w'
%
%     d = dot(unit(w), transl(T))
%     h = d / th
%
%     q = (transl(T) - h*th*w ) * inv(eye(3,3) - R)
%
%     v =
%     rho = (eye(3,3) - R')*t / 2 / (1-cos(th));
%
%     v = cross(rho, w);
%
%     tw = [skew(unit(w)) v'; 0 0 0  0];