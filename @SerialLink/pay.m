%PAY Joint forces from payload for SerialLink objects
%
% Calculates the joint loads due to a payload for SerialLink objects,
% It uses the formula Q = J'w, where w is a wrench vector applied at
% the end effector, w = [Fx Fy Fz Mxx Myy Mzz]'. The Jacobian can be
% supplied or computed by RTB
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) tauP = pay(w, J)
%  (2) tauP = robot.pay(q, w, f)
%
%  (1) Uses a supplied Jacobian
%  (2) Calculates the Jacobian for joint configuration q in frame f,
%       using either robot.jacob0(q) or robot.jacob0(q) for Jacobian
%
% Outputs:
%  tauP : Generalised joint force/torques
%
% Inputs:
%  robot : SerialLink object with n joints
%  w     : Wrench vector [Fx Fy Fz Mxx Myy Mzz]' in same frame as J
%  J     : Jacobian (supplied): 6-by-n or 6-by-n-m for trajectory
%  q     : Joint row vector, or trajectory matrix of joint row vectors
%  f     : '0' for world frame, 'n' for end-effector frame
%
% See also jacob0, jacobn, paycap, SerialLink.payload

% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.
%
% RTB LIBRARY:
%
% Copyright (C) 1993-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license

function tauP = pay(varargin)

if length(varargin) == 2
    w = varargin{1};
    J = varargin{2};
    n = size(J,2);
elseif length(varargin) == 4
    robot = varargin{1};
    q = varargin{2};
    w = varargin{3};
    f = varargin{4};
    n = robot.n;
    J = zeros(6,n,size(q,1));
    if f == '0'
        for i= 1: size(q,1)
            J(:,:,i) = robot.jacob0(q(i,:));
        end
    elseif f == 'n'
        for i= 1: size(q,1)
            J(:,:,i) = robot.jacobn(q(i,:));
        end
    end
end

tauP = -reshape(J(:,:)'*w,n,[])';

end

