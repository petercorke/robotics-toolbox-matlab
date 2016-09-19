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

classdef SO2 < RTBPose
    
    
    properties (Dependent = true)
        %R
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
            elseif isreal(t) && isvector(t)
                t = t(:)';
                if nargin > 1 && strcmp(deg, 'deg')
                    t = t *pi/180;
                end
                for i=1:length(t)
                    th = t(i);
                    obj(i).data = [
                        cos(th)  -sin(th)
                        sin(th)   cos(th)
                        ];
                end
            elseif isa(t, 'SO2')
                obj.data = t.data;
            elseif SO2.isa(t)
                for i=1:size(t, 3)
                    obj(i).data = t(:,:,i);
                end
            elseif SE2.isa(t)
                for i=1:size(t, 3)
                    obj(i).data = t(1:2,1:2,i);
                end

                
            end
        end
        
        function d = double(obj)
            d = obj.data;
        end
        
        function RR = R(obj)
            if ~isa(obj.data, 'sym')
                RR = zeros(2,2,length(obj)); % prealloc so long as not symbolic
            end
            for i=1:length(obj)
                RR(:,:,i) = obj(i).data(1:2,1:2);
            end
        end
        
        function TT = T(obj)
            TT = zeros(3,3,length(obj));
            for i=1:length(obj)
                TT(1:2,1:2,i) = obj(i).data(1:2,1:2);
                TT(3,3,i) = 1;
            end
        end
        
        
        function out = mtimes(a, b)
            if isa(b, 'SO2')
                % SO2 * SO2
                out = repmat(SO2, 1, max(length(a),length(b)));
                if length(a) == length(b)
                    % do objvector*objvector and objscalar*objscalar case
                    for i=1:length(a)
                        out(i) = SO2( a(i).data * b(i).data);
                    end
                elseif length(a) == 1
                    % objscalar*objvector case
                    for i=1:length(b)
                        out(i) = SO2( a.data * b(i).data);
                    end
                elseif length(b) == 1
                    % objvector*objscalar case
                    for i=1:length(a)
                        out(i) = SO2( a(i).data * b.data);
                    end
                else
                    error('RTB:SO2:badops', 'invalid operand lengths to * operator');
                end

                
            elseif numrows(b) == 2
                % SO2 * vectors (2xN), result is 2xN
                
                out = zeros(2, max(length(a),numcols(b)));  % preallocate space
                
                if length(a) == numcols(b)
                    % do objvector*vector and objscalar*scalar case
                    for i=1:length(a)
                        out(:,i) = a(i).data * b(:,i);
                    end
                elseif length(a) == 1
                    % objscalar*vector case
                    for i=1:length(a)
                        out = a.data * b;
                    end
                elseif numcols(b) == 1
                    % objvector*scalar case
                    for i=1:length(a)
                        out(:,i) = a(i).data * b;
                    end
                else
                    error('RTB:SO2:badops', 'invalid operand lengths to * operator');
                end     
            else
                error('RTB:SO2:badops', 'invalid operand types to * operator');
            end
        end
        
                function out = mrdivide(obj, a)
            assert( isa(a, 'SO2'), 'right-hand argument must be SO2');
            
            if isa(a, 'SO2')
                % SO2 / SO2
                out = repmat(SO2, 1, max(length(obj),length(a)));
                if length(obj) == length(a)
                    % do vector*vector and scalar*scalar case
                    for i=1:length(obj)
                        out(i) = SO2( obj(i).data * inv(a(i).data));
                    end
                elseif length(obj) == 1
                    % scalar*vector case
                    for i=1:length(obj)
                        out(i) = SO2( inv(obj.data) * a(i).data);
                    end
                elseif length(a) == 1
                    % vector*scalar case
                    for i=1:length(obj)
                        out(i) = SO2( obj(i).data * inv(a.data));
                    end
                else
                    error('RTB:SO2:badops', 'invalid operand lengths to / operator');
                end
                
            else
                error('RTB:SO2:badops', 'invalid operand types to / operator');
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
        
        
        function S = log(obj)
            S = logm(obj.data);
        end
        
        %         function o = set.R(obj, data)
        %             if isa(data, 'sym')
        %                 obj.data = data;
        %             else
        %             obj.data(1:2,1:2) = data;
        %             end
        %             o = obj;
        %         end
        
        function s = char(obj)
            s = num2str(obj.data, 4);
        end
        
        
        function T = SE2(obj)
            T = SE2( r2t(obj.data) );
        end
        
    end
    methods (Static)
        % Static factory methods for constructors from exotic representations
        
        function obj = exp(s)
            obj = SO2( trexp2(s) );
        end
        
        function n = new(obj, varargin)
            n = SO2(varargin{:});
        end
        
        function R = check(tr)
            if isa(tr, 'SO2')
                R = SO2(tr);        % enforce it being an SO2
            elseif SO2.isa(tr)
                R = SO2(tr);
            elseif SE2.isa(tr)
                R = SO2( t2r(tr) );
            else
                error('expecting an SO2 or 2x2 matrix');
            end
        end
        
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

