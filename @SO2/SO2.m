%SE2 Create planar translation and rotation transformation
%
% T = SE2(X, Y, THETA) is an SE(2) homogeneous transformation (3x3)
% representing translation X and Y, and rotation THETA in the plane.
%
% T = SE2(XY) as above where XY=[X,Y] and rotation is zero
%
% T = SE2(XY, THETA) as above where XY=[X,Y]
%
% T = SE2(XYT) as above where XYT=[X,Y,THETA]
%
% See also TRANSL2, TROT2, ISHOMOG2, TRPLOT2.


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

classdef SO2
    properties
        data
    end
    
    properties (Dependent = true)
        R
    end
    
    methods
        
        %ROT2 SO(2) Rotation matrix
        %
        % R = ROT2(THETA) is an SO(2) rotation matrix representing a rotation of THETA
        % radians.
        %
        % R = ROT2(THETA, 'deg') as above but THETA is in degrees.
        %
        % See also SE2, TROT2, ISROT2, TRPLOT2, ROTX, ROTY, ROTZ.
        
        function obj = SO2(t, deg)
            
            if nargin == 0
                obj.data = eye(2,2);
            elseif SO2.isa(t)
                obj.data = t;
            else
                
                if nargin > 1 && strcmp(deg, 'deg')
                    t = t *pi/180;
                end
                
                ct = cos(t);
                st = sin(t);
                obj.R = [
                    ct  -st
                    st   ct
                    ];
            end
        end
        
        function d = double(obj)
            d = obj.data;
        end
        
        function out = mtimes(obj, a)
            if isa(a, 'SO2')
                out = SO2( obj.data * a.data);
            elseif SO2.isa(a)
                out = SO2( obj.data * a);
                
            elseif isvec(a,2)
                out = obj.data * a;
            else
                error('bad thing');
            end
        end
        
        function th = theta(obj)
            th = atan2(obj.data(2,1), obj.data(1,1));
        end
        
        function ir = inv(obj)
            ir = SO2(obj.data');
        end
        
        function d = det(obj)
            d = det(obj.R);
        end
        
        function varargout = eig(obj, varargin)
            [varargout{1:nargout}] = eig(obj.data, varargin{:});
        end
        
        function R = get.R(obj)
            R = obj.data(1:2,1:2);
        end
        
        function o = set.R(obj, data)
            obj.data(1:2,1:2) = data;
            o = obj;
        end
        function display(l)
            %Link.display Display parameters
            %
            % L.display() displays the link parameters in compact single line format.  If L is a
            % vector of Link objects displays one line per element.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Link object and the command has no trailing
            %   semicolon.
            %
            % See also Link.char, Link.dyn, SerialLink.showlink.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(l) );
        end % display()
        function s = char(obj)
            s = num2str(obj.data, 4);
        end
    end
    methods (Static)
        
        %ISROT2 Test if SO(2) rotation matrix
        %
        % ISROT2(R) is true (1) if the argument is of dimension 2x2 or 2x2xN, else false (0).
        %
        % ISROT2(R, 'valid') as above, but also checks the validity of the rotation
        % matrix.
        %
        % Notes::
        % - A valid rotation matrix has determinant of 1.
        %
        % See also ISHOMOG2, ISVEC.
        
        function h = isa(r, dtest)
            
            d = size(r);
            if ndims(r) >= 2
                h =  all(d(1:2) == [2 2]);
                
                if h && nargin > 1
                    h = abs(det(r) - 1) < eps;
                end
            else
                h = false;
            end
        end
    end
end

