%TREXP matrix exponential for so(3) and se(3)
%
% For so(3)::
%
% R = TREXP(OMEGA) is the matrix exponential (3x3) of the so(3) element OMEGA that
% yields a rotation matrix (3x3). 
%
% R = TREXP(OMEGA, THETA) as above, but so(3) motion of THETA*OMEGA.
%
% R = TREXP(S, THETA) as above, but rotation of THETA about the unit vector S.
%
% R = TREXP(W) as above, but the so(3) value is expressed as a vector W
% (1x3) where W = S * THETA. Rotation by ||W|| about the vector W.
%
% For se(3)::
%
% T = TREXP(SIGMA) is the matrix exponential (4x4) of the se(3) element SIGMA that
% yields a homogeneous transformation  matrix (4x4). 
%
% T = TREXP(TW) as above, but the se(3) value is expressed as a twist vector TW
% (1x6). 
%
% T = TREXP(SIGMA, THETA) as above, but se(3) motion of SIGMA*THETA, the
% rotation part of SIGMA (4x4) must be unit norm.
%
% T = TREXP(TW, THETA) as above, but se(3) motion of TW*THETA, the
% rotation part of TW (1x6) must be unit norm.
%
% Notes::
% - Efficient closed-form solution of the matrix exponential for arguments that are
%   so(3) or se(3).
% - If theta is given then the first argument must be a unit vector or a
%   skew-symmetric matrix from a unit vector.
% - Angle vector argument order is different to ANGVEC2R.
%
% References::
% - Robotics, Vision & Control: Second Edition, Chap 2,
%   P. Corke, Springer 2016.
% - "Mechanics, planning and control"
%   Park & Lynch, Cambridge, 2017.
%
% See also ANGVEC2R, TRLOG, TREXP2, SKEW, SKEWA, Twist.


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
function T = trexp(S, theta)

    if ishomog(S) || isvec(S,6)
        % input is se(3)
        
        if nargin == 1
            % twist vector 1x6 or augmented skew matrix 4x4
            if isvec(S,6)
                % it's a twist vector
                S = skewa(S);
            end
            T = expm(S);
        else
            % se(3) plus twist
            if all(size(S) == 4)
                % it's se(3) matrix
                [skw,v] = tr2rt(S);
            else
                % it's a twist vector
                v = S(1:3); v= v(:);
                skw = skew(S(4:6));
            end
            
            % use an efficient solution
            R = trexp(skw, theta);
            t = (eye(3,3)*theta + (1-cos(theta))*skw + (theta-sin(theta))*skw^2)*v;
            
            T = rt2tr(R,t);
        end         
    elseif ishomog2(S) || isvec(S,3)
        % input is so(3)
        
        if isrot(S)
            % input is 3x3 skew symmetric
            w = vex(S);
        elseif isvec(S)
            % input is a 3-vector
             w = S;
        else
            error('RTB:trexp:badarg', 'expecting 1x3 or 3x3');     
        end
        
        % for a zero so(3) return unit matrix, theta not relevant
        if norm(w) < 10*eps
            T = eye(3,3);
            return;
        end
        
        if nargin == 1
            %  theta is not given, extract it
            theta = norm(w);
            w = unit(w);
        else
            % theta is given
            assert(isunit(w), 'RTB:trexp:badarg',  'w must be a unit twist');
        end
        
        S = skew(w);
        
        T = eye(3,3) + sin(theta)*S + (1-cos(theta))*S^2;
        
    else
        error('RTB:trexp:badarg', 'first argument must be so(3), 3-vector, se(3) or 6-vector');
    end
end