%RPY2JAC Jacobian from RPY angle rates to angular velocity
%
% J = RPY2JAC(RPY, OPTIONS) is a Jacobian matrix (3x3) that maps ZYX roll-pitch-yaw angle 
% rates to angular velocity at the operating point RPY=[R,P,Y].
%
% J = RPY2JAC(R, P, Y, OPTIONS) as above but the roll-pitch-yaw angles are passed
% as separate arguments.
%
% Options::
% 'xyz'     Use XYZ roll-pitch-yaw angles
% 'yxz'     Use YXZ roll-pitch-yaw angles
%
% Notes::
% - Used in the creation of an analytical Jacobian.
%
% See also EUL2JAC, SerialLink.JACOBE.




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

function J = rpy2jac(r, varargin)

    opt.order = {'zyx', 'xyz', 'yxz'};
    [opt,args] = tb_optparse(opt, varargin);
    
    
        % unpack the arguments
    if numcols(r) == 3
		p = r(:,2);
		y = r(:,3);
		r = r(:,1);
	elseif nargin >= 3
        p = args{1};
        y = args{2};
    else
        error('RTB:rpy2jac:badarg', 'bad arguments')
    end
    
    
    switch opt.order
    case 'xyz'
        J = [	
        sin(p)          0       1  
        -cos(p)*sin(y)  cos(y)  0  
        cos(p)*cos(y)   sin(y)  0  
        ];
    
     case 'zyx'
        J = [ 
            cos(p)*cos(y), -sin(y), 0
            cos(p)*sin(y),  cos(y), 0
            -sin(p),       0, 1
            ];
    
    case 'yxz'
        J = [
            cos(p)*sin(y),  cos(y), 0
            -sin(p),       0, 1
            cos(p)*cos(y), -sin(y), 0
            ];
    end
    
%{
    syms r p y rd pd yd wx wy wz real
    syms rt(t) pt(t) yt(t) 

    order = 'yxz'

    R = rpy2r(r, p, y, order);
    Rt = rpy2r(rt, pt, yt, order);
    dRdt = diff(Rt, t);
    dRdt = subs(dRdt, {diff(rt(t),t), diff(pt(t),t), diff(yt(t),t),}, {rd,pd,yd});
    dRdt = subs(dRdt, {rt(t),pt(t),yt(t)}, {r,p,y});
    dRdt = formula(dRdt)   % convert symfun to an array

    w = vex(dRdt * R');
    w = simplify(w)

    clear A
    rpyd = [rd pd yd];

    for i=1:3
        for j=1:3
            C = coeffs(w(i), rpyd(j));
            if length(C) == 1
                A(i,j) = 0;
            else
            A(i,j) = C(2);
            end
        end
    end

    A
%}