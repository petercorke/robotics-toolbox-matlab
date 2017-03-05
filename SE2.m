%SE2 Representation of 2D rigid-body motion
%
% This subclasss of SO2 < RTBPose is an object that represents an SE(2)
% rigid-body motion.
%
% Constructor methods::
%  SE2          general constructor
%  SE2.exp      exponentiate an se(2) matrix  
%  SE2.rand     random transformation
%  new          new SE2 object
%
% Information and test methods::
%  dim*         returns 2
%  isSE*        returns true
%  issym*       true if rotation matrix has symbolic elements
%  isa          check if matrix is SE2
%
% Display and print methods::
%  plot*        graphically display coordinate frame for pose
%  animate*     graphically animate coordinate frame for pose
%  print*       print the pose in single line format
%  display*     print the pose in human readable matrix form
%  char*        convert to human readable matrix as a string
%
% Operation methods::
%  det          determinant of matrix component
%  eig          eigenvalues of matrix component
%  log          logarithm of rotation matrix
%  inv          inverse
%  simplify*    apply symbolic simplication to all elements
%  interp       interpolate between poses
%
% Conversion methods::
%  check        convert object or matrix to SE2 object
%  theta        return rotation angle
%  double       convert to rotation matrix
%  R            convert to rotation matrix
%  SE2          convert to SE2 object with zero translation
%  T            convert to homogeneous transformation matrix
%  t            translation column vector
%
% Compatibility methods::
%  isrot2*      returns false
%  ishomog2*    returns true
%  tr2rt*       convert to rotation matrix and translation vector
%  t2r*         convert to rotation matrix
%  trprint2*    print single line representation
%  trplot2*     plot coordinate frame
%  tranimate2*  animate coordinate frame
%  transl2      return translation as a row vector  
%
% Static methods::
%  check        convert object or matrix to SO2 object

%
% * means inherited from RTBPose
%
% Operators::
%  +           elementwise addition, result is a matrix
%  -           elementwise subtraction, result is a matrix
%  *           multiplication within group, also group x vector
%  /           multiply by inverse
%  ==          test equality
%  ~=          test inequality
%
% See also SE2, SO3, SE3, RTBPose.



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


classdef SE2 < SO2
    
    properties (Dependent = true)
        t
    end
    
    methods
        
        
        function obj = SE2(varargin)
            %SE2.SE2 Construct an SE(2) object
            %
            % Constructs an SE(2) pose object that contains a 3x3 homogeneous transformation
            % matrix.
            %
            % T = SE2() is a null relative motion
            %
            % T = SE2(X, Y) is an object representing pure translation defined by X and
            % Y
            %
            % T = SE2(XY) is an object representing pure translation defined by XY
            % (2x1). If XY (Nx2) returns an array of SE2 objects, corresponding to
            % the rows of XY.
            %
            % T = SE2(X, Y, THETA) is an object representing translation, X and Y, and
            % rotation, angle THETA.
            %
            % T = SE2(XY, THETA) is an object representing translation, XY (2x1), and
            % rotation, angle THETA
            %
            % T = SE2(XYT) is an object representing translation, XYT(1) and XYT(2),
            % and rotation, angle XYT(3). If XYT (Nx3) returns an array of SE2 objects, corresponding to
            % the rows of XYT.
            %
            % T = SE2(R) is an object representing pure rotation defined by the
            % orthonormal rotation matrix R (2x2)
            %
            % T = SE2(R, XY) is an object representing rotation defined by the
            % orthonormal rotation matrix R (2x2) and position given by XY (2x1)
            %
            % T = SE2(T) is an object representing translation and rotation defined by
            % the homogeneous transformation matrix T (3x3).  If T (3x3xN) returns an array of SE2 objects, corresponding to
            % the third index of T
            %
            % T = SE2(T) is an object representing translation and rotation defined by
            % the SE2 object T, effectively cloning the object. If T (Nx1) returns an array of SE2 objects, corresponding to
            % the index of T
            %
            % Options::
            % 'deg'         Angle is specified in degrees
            %
            % Notes::
            % - Arguments can be symbolic
            % - The form SE2(XY) is ambiguous with SE2(R) if XY has 2 rows, the second form is assumed.
            % - The form SE2(XYT) is ambiguous with SE2(T) if XYT has 3 rows, the second form is assumed.
            
            opt.deg = false;
            
            [opt,args] = tb_optparse(opt, varargin);
            
            if opt.deg
                scale = pi/180.0;
            else
                scale = 1;
            end
            
            % if any of the arguments is symbolic the result will be symbolic
            if any( cellfun(@(x) isa(x, 'sym'), args) )
                obj.data = sym(obj.data);
            end
            
            obj.data = eye(3,3);

            switch length(args)
                case 0
                    % null motion
                    return
                case 1
                    % 1 argument
                    a = args{1};
                    
                    if isvec(a, 2)
                        % (t)
                        obj.data = [ 1 0 a(1); 0 1 a(2); 0 0 1];
                        
                    elseif isvec(a, 3)
                        % ([x y th])
                        a = a(:);
                        obj.data(1:2,1:2) = rot2(a(3)*scale);
                        obj.t = a(1:2);
                        
                    elseif SO2.isa(a)
                        % (R)
                        obj.data(1:2,1:2) = a;
                        
                    elseif SE2.isa(a)
                        % (T)
                        for i=1:size(a, 3)
                            obj(i).data = a(:,:,i);
                        end
                    elseif isa(a, 'SE2')
                        % (SE2)
                        for i=1:length(a)
                            obj(i).data = a(i).data;
                        end
                        
                    elseif any( numcols(a) == [2 3] )
                        for i=1:numrows(a)
                            obj(i) = SE2(a(i,:));
                        end
                        return
                    else
                        error('RTB:SE2:badarg', 'unknown arguments');
                    end
                    
                case 2
                    % 2 arguments
                    a = args{1}; b = args{2};
                    if isscalar(a) && isscalar(b)
                        % (x,y)
                        obj.data = [ 1 0 a; 0 1 b; 0 0 1];
                    elseif isvec(a,2) && isscalar(b)
                        % ([x y], th)
                        obj.data = [ rot2(b*scale) a(:); 0 0 1];
                    elseif SO2.isa(a) && isvec(b,2)
                        % (R, t)
                        obj.data = [a b(:); 0 0 1];
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                    
                case 3
                    % 3 arguments
                    a = args{1}; b = args{2}; c = args{3};
                    if isscalar(a) && isscalar(b) && isscalar(c)
                        % (x, y, th)
                        obj.data = [ rot2(c*scale) [a b]'; 0 0 1];
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                otherwise
                    error('RTB:SE3:badarg', 'unknown arguments');
                    
            end
            
            % add the last row if required
%             if numrows(obj.data) == 2
%                 obj.data = [obj.data; 0 0 1];
%             end
            assert(all(size(obj(1).data) == [3 3]), 'RTB:SE2:SE2', 'created wrong size data element');
            %% HACK
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  GET AND SET
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function t = get.t(obj)
            %SE2.t  Get translational component
            %
            % P.t is a column vector (2x1) representing the translational component of
            % the rigid-body motion described by the SE2 object P.
            %
            % Notes::
            % - If P is a vector the result is a MATLAB comma separated list, in this
            % case use P.transl().
            %
            % See also SE2.transl.
            t = obj.data(1:2,3);
        end
        
        function o = set.t(obj, t)
            %SE2.t  Set translational component
            %
            % P.t = TV sets the translational component of the rigid-body motion
            % described by the SE2 object P to TV (2x1).
            %
            % Notes::
            % - TV can be a row or column vector.
            % - If TV contains a symbolic value then the entire matrix is converted to
            %   symbolic.
            
            if isa(t, 'sym') && ~isa(obj.data, 'sym')
                obj.data = sym(obj.data);
            end
            obj.data(1:2,3) = t;
            o = obj;
        end
        
        function t = transl(obj)
            %SE2.t  Get translational component
            %
            % TV = P.transl() is a row vector (1x2) representing the translational component of
            % the rigid-body motion described by the SE2 object P.  If P is a vector of
            % objects (1xN) then TV (Nx2) will have one row per object element.
            
            t = obj.t';
        end
        
        function T = T(obj)
            %SE2.T  Get homogeneous transformation matrix
            %
            % T = P.T() is the homogeneous transformation matrix (3x3) associated with the
            % SE2 object P, and has zero translational component.  If P is a vector
            % (1xN) then T (3x3xN) is a stack of rotation matrices, with the third
            % dimension corresponding to the index of P.
            %
            % See also SO2.T.
            for i=1:length(obj)
                T(:,:,i) = obj(i).data;
            end
        end
        
        function t = SE3(obj)
            %SE2.SE3 Lift to 3D
            %
            % Q = P.SE3() is an SE3 object formed by lifting the rigid-body motion
            % described by the SE2 object P from 2D to 3D.  The rotation is about the
            % z-axis, and the translational is within the xy-plane.
            %
            % See also SE3.
            t = SE3();
            t.data(1:2,1:2) = obj.data(1:2,1:2);
            t.data(1:2,4) = obj.data(1:2,3);
        end
        
        function out = SO2(obj)
            %SE2.SO2  Extract SO(2) rotation
            %
            % Q = SO2(P) is an SO2 object that represents the rotational component of
            % the SE2 rigid-body motion.
            %
            % See also SE2.R.
            
            out = SO2( obj.R );
        end
        
             
        
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  SE(2) OPERATIONS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function it = inv(obj)
            %SE2.inv  Inverse of SE2 object
            %
            % Q = inv(P) is the inverse of the SE2 object P.  P*Q will be the identity
            % matrix.
            %
            % Notes::
            % - This is formed explicitly, no matrix inverse required.   
            it = SE2( obj.R', -obj.R'*obj.t);
        end
 
        
        function v = xyt(obj)
            %SE2.xyt  Construct SE2 object from Lie algebra
            %
            % XYT = P.xyt() is a column vector (3x1) comprising the minimum three
            % parameters of this rigid-body motion [x; y; theta] with translation (x,y)
            % and rotation theta.

            
            % TODO VECTORISE
            v = obj.t;
            v(3) = atan2(obj.data(2,1), obj.data(1,1));
        end

        function S = log(obj)
            %SE2.log  Lie algebra
            %
            % se2 = P.log() is the Lie algebra augmented skew-symmetric matrix (3x3)
            % corresponding to the SE2 object P.
            %
            % See also SE2.Twist, logm.
            S = logm(obj.data);
        end
        
        function T = interp(obj1, obj2, s)
            %SE2.interp Interpolate between SO2 objects
            %
            % P1.interp(P2, s) is an SE2 object representing interpolation
            % between rotations represented by SE3 objects P1 and P2.  s varies from 0
            % (P1) to 1 (P2). If s is a vector (1xN) then the result will be a vector
            % of SE2 objects.
            %
            % Notes::
            % - It is an error if S is outside the interval 0 to 1.
            %
            % See also SO2.angle.
            assert(all(s>=0 & s<=1), 'RTB:SE2:interp:badarg', 's must be in the interval [0,1]');
            
            th1 = obj1.angle; th2 = obj2.angle;
            xy1 = obj1.t; xy2 = obj2.t;
            
            T = SE2( xy1 + s*(xy2-xy1), th1 + s*(th2-th1) );
        end

        function tw = Twist(obj)
            %SE2.Twist  Convert to Twist object
            %
            % TW = P.Twist() is the equivalent Twist object.  The elements of the twist are the unique
            % elements of the Lie algebra of the SE2 object P.
            %
            % See also SE2.log, Twist.
            tw = Twist( obj.log );
        end
        
         
        function print(obj, varargin)
            for T=obj
                theta = atan2(T.data(2,1), T.data(1,1)) * 180/pi;
                fprintf('t = (%.4g, %.4g), theta = %.4g deg\n', T.t, theta);
            end
        end
        
        function n = new(obj, varargin)
            %SE2.new  Construct a new object of the same type
            %
            % P2 = P.new(X) creates a new object of the same type as P, by invoking the SE2 constructor on the matrix
            % X (3x3).
            %
            % P2 = P.new() as above but defines a null motion.
            %
            % Notes::
            % - Serves as a dynamic constructor.
            % - This method is polymorphic across all RTBPose derived classes, and
            %   allows easy creation of a new object of the same class as an existing
            %   one.
            %
            % See also SE3.new, SO3.new, SO2.new.
            
            n = SE2(varargin{:});
        end
        
    end
    
    methods (Static)
        % Static factory methods for constructors from exotic representations
        
        function obj = exp(s)
            %SE2.exp  Construct SE2 object from Lie algebra
            %
            % P = SE2.exp(se2) creates an SE2 object by exponentiating the se(2)
            % argument (3x3).
            obj = SE2( trexp2(s) );
        end
        

        
        function T = check(tr)
            %SE2.check  Convert to SE2
            %
            % Q = SE2.check(X) is an SE2 object where X is SE2 or 3x3
            % homogeneous transformation matrix.
            if isa(tr, 'SE2')
                T = tr;
            elseif SE2.isa(tr)
                T = SE2(tr);
            else
                error('expecting an SE2 or 3x3 matrix');
            end
        end
        
        function h = isa(tr, rtest)
            %SE2.ISA Test if matrix is SE(2)
            %
            % SE2.ISA(T) is true (1) if the argument T is of dimension 3x3 or 3x3xN, else
            % false (0).
            %
            % SE2.ISA(T, true') as above, but also checks the validity of the rotation
            % sub-matrix.
            %
            % Notes::
            % - The first form is a fast, but incomplete, test for a transform in SE(3).
            % - There is ambiguity in the dimensions of SE2 and SO3 in matrix form.
            %
            % See also SO3.ISA, SE2.ISA, SO2.ISA, ishomog2.
            
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
                
        function T = rand()
            %SE2.rand Construct a random SE(2) object
            %
            % SE2.rand() is an SE2 object with a uniform random translation and a
            % uniform random orientation.  Random numbers are in the interval 0 to
            % 1.
            %
            % See also RAND.
            T = SE2(rand(1,3));
        end
    end
end
