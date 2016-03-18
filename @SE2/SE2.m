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
        % ()
        % (T)
        % (t)
        % ([x y th])
        % (R)
        % (R, t)
        % (x, y)
        % ([x y], th)
        % (x, y, th)
        
        function obj = SE2(varargin)
            obj.data = eye(3,3);
            
            opt.deg = false;
            
            [opt,args] = tb_optparse(opt, varargin);
            
            if opt.deg
                scale = pi/180.0;
            else
                scale = 1;
            end
            
            switch length(args)
                case 0
                    return
                case 1
                    a = args{1};
                    if isvec(a, 2)
                        % (t)
                        obj.t = a(:);
                        obj.data(3,:) = [0 0 1];
                    elseif SO2.isa(a)
                        % (R)
                        obj.data = r2t(a);
                        
                        
                    elseif SE2.isa(a)
                        % (T)
                        for i=1:size(a, 3)
                            obj(i).data = a(:,:,i);
                        end
                    elseif isa(a, 'SE2')
                        % (SE3)
                        for i=1:length(a)
                            obj(i).data = a(i).data;
                        end
                        
                    elseif isvec(a, 3)
                        % ([x y th])
                        obj.data(1:2,1:2) = rot2(a(3)*scale);
                        obj.data(1:2,3) = a(1:2)';
                    else
                        error('RTB:SE2:badarg', 'unknown arguments');
                    end
                case 2
                    a = args{1}; b = args{2};
                    if isscalar(a) && isscalar(b)
                        % (x,y)
                        obj.data(1,3) = a;
                        obj.data(2,3) = b;
                    elseif isvec(a,2) && isscalar(b)
                        % ([x y], th)
                        obj.data(1:2,1:2) = rot2(b*scale);
                        obj.data(1:2,3) = a;
                    elseif SO2.isa(a) && isvec(b,2)
                        % (R, t)
                        obj.data(1:2,1:2) = a;
                        obj.data(1:2,3) = b;
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                case 3
                    a = args{1}; b = args{2}; c = args{3};
                    if isscalar(a) && isscalar(b) && isscalar(c)
                        % (x, y, th)
                        obj.data(1,3) = a;
                        obj.data(2,3) = b;
                        obj.data(1:2,1:2) = rot2(c*scale);
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                otherwise
                    error('RTB:SE3:badarg', 'unknown arguments');
                    
            end
            
        end
        
        function out = mtimes(obj, a)
            % vectorise
            if isa(obj, 'SE2') && isa(a, 'SE2')
                out = SE2( obj.data * a.data);
            elseif ishomog2(obj) && isa(a, 'SE2')
                out = SE2( obj * a.data);
                
            elseif isa(obj, 'SE2') && ishomog2(a)
                out = SE2( obj.data * a);
                
                
            elseif SE2.isa(a)
                % vectorise
                out = SE2( obj.data * a);
                
            elseif isreal(a) && numrows(a) == 2
                out = obj.data * [a; ones(1, numcols(a))];
                out = out(1:2,:);
                
            elseif isvec(a,3)
                out = obj.data * a(:);
            else
                error('bad thing');
            end
        end
        
                function out = mrdivide(obj, a)
            assert( isa(a, 'SE2'), 'right-hand argument must be SE2');
            
            if isa(a, 'SE2')
                % SE2 / SE2
                out = repmat(SE2, 1, max(length(obj),length(a)));
                if length(obj) == length(a)
                    % do vector*vector and scalar*scalar case
                    for i=1:length(obj)
                        out(i) = SE2( obj(i).data * inv(a(i).data));
                    end
                elseif length(obj) == 1
                    % scalar*vector case
                    for i=1:length(obj)
                        out(i) = SE2( inv(obj.data) * a(i).data);
                    end
                elseif length(a) == 1
                    % vector*scalar case
                    for i=1:length(obj)
                        out(i) = SE2( obj(i).data * inv(a.data));
                    end
                else
                    error('RTB:SE2:badops', 'invalid operand lengths to / operator');
                end
                
            else
                error('RTB:SE2:badops', 'invalid operand types to / operator');
            end
                end
        
        
        
        function print(obj, varargin)
            for T=obj
                theta = atan2(T.data(2,1), T.data(1,1)) * 180/pi;
                fprintf('t = (%.4g, %.4g), theta = %.4g deg\n', T.t, theta);
            end
        end
        
        function it = inv(obj)
            it = SE2( obj.R', -obj.R'*obj.t);
        end
        function t = get.t(obj)
            t = obj.data(1:2,3);
        end
        function o = set.t(obj, t)
            if isa(t, 'sym') && ~isa(obj.data, 'sym')
                obj.data = sym(obj.data);
            end
            obj.data(1:2,3) = t;
            o = obj;
        end

        
        function out = simplify(obj)
            out = obj;
            if isa(obj.data, 'sym')
                out.data = simplify(out.data);
            end
        end
        
        function v = xyt(obj)
            % VECTORISE
            v = obj.t;
            v(3) = atan2(obj.R(2,1), obj.R(1,1));
        end
        
        function T = T(obj)
            for i=1:length(obj)
                T(:,:,i) = obj(i).data;
            end
        end
        function S = log(obj)
            S = logm(obj.data);
        end
        
        function t = SE3(obj)
            t = SE3();
            t.data(1:2,1:2) = obj.data(1:2,1:2);
            t.data(1:2,4) = obj.data(1:2,3);
        end
        
        function tw = Twist(obj)
            tw = Twist( obj.log );
        end
        
        function out = SO2(obj)
            out = SO2( obj.R );
        end
    end
    
    methods (Static)
        % Static factory methods for constructors from exotic representations
        function obj = exp(s)
            obj = SE2( trexp2(s) );
        end
        function n = new(obj, varargin)
            n = SE2(varargin{:});
        end
        
        function T = check(tr)
            if isa(tr, 'SE2')
                T = tr;
            elseif SE2.isa(tr)
                T = SE2(tr);
            else
                error('expecting an SE2 or 3x3 matrix');
            end
        end
        
        function h = isa(tr, rtest)
            %SE2.ISA Test if a homogeneous transformation
            %
            % SE2.ISA(T) is true (1) if the argument T is of dimension 3x3 or 3x3xN, else
            % false (0).
            %
            % SE2.ISA(T, 'valid') as above, but also checks the validity of the rotation
            % sub-matrix.
            %
            % Notes::
            % - The first form is a fast, but incomplete, test for a transform in SE(3).
            % - There is ambiguity in the dimensions of SE2 and SO3 in matrix form.
            % See also SO3.ISA, SE2.ISA, SO2.ISA.
            d = size(tr);
            if ndims(tr) >= 2
                h =  all(d(1:2) == [3 3]);
                
                if h && nargin > 1
                    h = SO3.isa( tr(1:2,1:2) );
                end
            else
                h = false;
            end
        end
    end
end
