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

classdef SE2 < SO2
    
    properties (Dependent = true)
        t
    end
    
    methods
        
        function obj = SE2(a, b, c, varargin)
            obj.data = eye(3,3);
            
            if nargin == 0
                return;
            elseif nargin >= 2 && SO2.isa(a) && isvec(b,2)
                obj.R = a;
                obj.t = b;
                obj.data(3,:) = [0 0 1];
                
            elseif isvec(a, 2)
                obj.t = a(:);
                                obj.data(3,:) = [0 0 1];

            else
                
                opt.deg = false;
                
                opt = tb_optparse(opt, varargin);
                
                if length(a) == 3
                    x = a(1);
                    y = a(2);
                    th = a(3);
                elseif length(a) == 2
                    x = a(1);
                    y = a(2);
                    if nargin < 2
                        th = 0;
                    else
                        th = b;
                    end
                else
                    x = a;
                    y = b;
                    if nargin < 3
                        th = 0;
                    else
                        th = c;
                    end
                end
                
                if opt.deg
                    th = th * pi/180.0;
                end
                cth = cos(th);
                sth = sin(th);
                R = [cth -sth; sth cth];
                obj.R = R;
                obj.t = [x;y];
                obj.data(3,:) = [0 0 1];
            end
        end
        
        function out = mtimes(obj, a)
            if isa(a, 'SE2')
                out = SE2( obj.data * a.data);
            elseif SE2.isa(a)
                out = SE2( obj.data * a);
                
            elseif isvec(a,2)
                out = obj.data * [a(:);1];
                out = out(1:2);
            else
                error('bad thing');
            end
        end
        
        function it = inv(obj)
            it = SE2( obj.R', -obj.R*obj.t);
        end
        function t = get.t(obj)
            t = obj.data(1:2,3);
        end
        function o = set.t(obj, t)
            obj.data(1:2,3) = t;
            o = obj;
        end
        
        %ISHOMOG2 Test if SE(2) homogeneous transformation
        %
        % ISHOMOG2(T) is true (1) if the argument T is of dimension 3x3 or 3x3xN, else
        % false (0).
        %
        % ISHOMOG2(T, 'valid') as above, but also checks the validity of the rotation
        % sub-matrix.
        %
        % Notes::
        % - The first form is a fast, but incomplete, test for a transform in SE(3).
        % - Does not work for the SE(3) case.
        %
        % See also ISHOMOG, ISROT2, ISVEC.
        
    end
    
    methods (Static)
        
        function h = isa(tr, rtest)
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
    end
end
