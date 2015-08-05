%SE3 Create planar translation and rotation transformation
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

%TODO
% interp
% animate
% chain
% animate
% jacobian
% log
% norm
% print
% 
% all vectorised!
% 
%
% Superclass, provides char,display, get/set R,t,T

classdef SE3 < SO3
    
    properties (Dependent = true)
        t
        T
    end
    
    methods
        
        function obj = SE3(a, b, c, varargin)
            obj.data = eye(4,4);
            if nargin == 0
                return;
            elseif SE3.isa(a)
                obj.data = a;
            elseif nargin >= 2 && SO3.isa(a) && isvec(b,3)
                obj.R = a;
                obj.t = b;
            elseif nargin >= 1 && SO3.isa(a)
                obj.R = a;
                obj.t = b;
            elseif nargin >= 1 && isvec(a, 3)
                obj.t = a(:);
        end
end
        

                function out = mtimes(obj, a)
            if isa(a, 'SE3')
                out = SE3( obj.data * a.data);
            elseif SE3.isa(a)
                out = SE3( obj.data * a);
                
            elseif isvec(a,3)
                out = obj.data * [a(:);1];
                out = out(1:3);
            else
                error('bad thing');
            end
        end
        
        function it = inv(obj)
            it = SE3( obj.R', -obj.R*obj.t);
        end
        function t = get.t(obj)
            t = obj.data(1:3,4);
        end
        function o = set.t(obj, t)
            obj.data(1:3,4) = t;
            o = obj;
        end
        function T = get.T(obj)
            T = obj.data;
        end
        function o = set.T(obj, T)
            obj.data = T;
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
        
        function obj = rotx(varargin)
            obj = SE3( SO3.rotx(varargin{:}) );
        end
        function obj = roty(varargin)
            obj = SE3( SO3.roty(varargin{:}) );
        end
        function obj = rotz(varargin)
            obj = SE3( SO3.rotz(varargin{:}) );
        end
        function obj = oa(varargin)
            obj = SE3( SO3.oa(varargin{:}) );
        end
        function obj = angvec(varargin)
            obj = SE3( SO3.angvec(varargin{:}) );
        end
        function obj = rpy(varargin)
            obj = SE3( SO3.rpy(varargin{:}) );
        end
        function obj = eul(varargin)
            obj = SE3( SO3.eul(varargin{:}) );
        end
        
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
