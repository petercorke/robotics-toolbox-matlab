%SO2 Representation of 2D rotation
%
% This subclasss of RTBPose is an object that represents an SO(2) rotation
%
% Constructor methods::
%  SO2          general constructor
%  SO2.exp      exponentiate an so(2) matrix
%  SO2.rand     random orientation
%  new          new SO2 object
%
% Information and test methods::
%  dim*         returns 2
%  isSE*        returns false
%  issym*       true if rotation matrix has symbolic elements
%  isa          check if matrix is SO2
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
%  interp       interpolate between rotations
%
% Conversion methods::
%  check        convert object or matrix to SO2 object
%  theta        return rotation angle
%  double       convert to rotation matrix
%  R            convert to rotation matrix
%  SE2          convert to SE2 object with zero translation
%  T            convert to homogeneous transformation matrix with zero translation
%
% Compatibility methods::
%  isrot2*      returns true
%  ishomog2*    returns false
%  trprint2*    print single line representation
%  trplot2*     plot coordinate frame
%  tranimate2*  animate coordinate frame
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

classdef SO2 < RTBPose
    
    
    properties (Dependent = true)
        %R
    end
    
    methods
        
        function obj = SO2(t, deg)
            %SO2.SO2  Construct an SO(2) object
            %
            % P = SO2() is an SO2 object representing null rotation.
            %
            % P = SO2(THETA) is an SO2 object representing rotation of THETA radians.
            % If THETA is a vector (N) then P is a vector of objects, corresponding to
            % the elements of THETA.
            %
            % P = SO2(THETA, 'deg') as above but with THETA degrees.
            %
            % P = SO2(R) is an SO2 object formed from the rotation 
            % matrix R (2x2)
            %
            % P = SO2(T) is an SO2 object formed from the rotational part 
            % of the homogeneous transformation matrix T (3x3)
            %
            % P = SO2(Q) is an SO2 object that is a copy of the SO2 object Q.            %
            %
            % See also rot2, SE2, SO3.
            
            if nargin == 0
                % null rotation
                obj.data = eye(2,2);
            elseif isa(t, 'SO2')
                % copy an existing SO2 object
                obj.data = t.data;
            elseif isvector(t)
                % for specified angle
                if isfloat(t)
                    assert(isreal(t), 'argument cannot be complex');
                end
                t = t(:).';
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

            elseif SO2.isa(t)
                % from a 2x2 matrix
                for i=1:size(t, 3)
                    obj(i).data = t(:,:,i);
                end
            elseif SE2.isa(t)
                % from a 3x3 matrix
                for i=1:size(t, 3)
                    obj(i).data = t(1:2,1:2,i);
                end  
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  GET AND SET
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        
        function RR = R(obj)
            %SO2.R  Get rotation matrix
            %
            % R = P.R() is the rotation matrix (2x2) associated with the SO2 object P.  If P
            % is a vector (1xN) then R (2x2xN) is a stack of rotation matrices, with
            % the third dimension corresponding to the index of P.
            %
            % See also SO2.T.
            if ~issym(obj)
                RR = zeros(2,2,length(obj)); % prealloc so long as not symbolic
            end
            for i=1:length(obj)
                RR(:,:,i) = obj(i).data(1:2,1:2);
            end
        end
        
        function TT = T(obj)
            %SO2.T  Get homogeneous transformation matrix
            %
            % T = P.T() is the homogeneous transformation matrix (3x3) associated with the
            % SO2 object P, and has zero translational component.  If P is a vector
            % (1xN) then T (3x3xN) is a stack of rotation matrices, with the third
            % dimension corresponding to the index of P.
            %
            % See also SO2.T.
            TT = zeros(3,3,length(obj));
            for i=1:length(obj)
                TT(1:2,1:2,i) = obj(i).data(1:2,1:2);
                TT(3,3,i) = 1;
            end
        end
        
        function th = angle(obj)
            %SO2.theta  Rotation angle
            %
            % THETA = P.angle() is the rotation angle, in radians, associated with the
            % SO2 object P.
            th = atan2(obj.data(2,1), obj.data(1,1));
        end
  
        function th = theta(obj)
            %SO2.theta  Rotation angle
            %
            % THETA = P.theta() is the rotation angle, in radians, associated with the
            % SO2 object P.
            %
            % Notes::
            % - Deprecated, use angle() instead.
            
            % TODO: remove
            
            warning('RTB:SO2:deprecfun', 'the theta() method is deprecated use angle()');
            th = obj.angle();
        end

                
        function s = char(obj)
            %SO2.char Convert to string
            %
            % s = P.char() is a string containing rotation matrix elements.
            %
            % See also RTB.display.            
            s = num2str(obj.data, 4);
        end
        
        function T = SE2(obj)
            %SO2.SE2 Convert to SE2 object
            %
            % Q = P.SE2() is an SE2 object formed from the rotational component of the
            % SO2 object P and with a zero translational component.
            %
            % See also SE2.
            T = SE2( r2t(obj.data) );
        end
        
        


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  OPERATIONS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function ir = inv(obj)
            %SO2.inv  Inverse of SO2 object
            %
            % Q = inv(P) is the inverse of the SO2 object P.  P*Q will be the identity
            % matrix.
            %
            % Notes::
            % - This is simply the transpose of the matrix.
            ir = SO2(obj.data');
        end
        
        function d = det(obj)
            %SO2.inv  Determinant of SO2 object
            %
            % det(P) is the determinant of the SO2 object P and should always be +1.
            d = det(obj.R);
        end
        
        function varargout = eig(obj, varargin)
            %SO2.eig  Eigenvalues and eigenvectors
            %
            % E = eig(P) is a column vector containing the eigenvalues of the the
            % rotation matrix of the SO2 object P.
            %
            % [V,D] = eig(P) produces a diagonal matrix D of eigenvalues and 
            % a full matrix V whose columns are the corresponding eigenvectors  
            % so that A*V = V*D.
            %
            % See also eig.
            [varargout{1:nargout}] = eig(obj.data, varargin{:});
        end
            
        function S = log(obj)
            %SO2.log  Lie algebra
            %
            % so2 = P.log() is the Lie algebra skew-symmetric matrix (2x2)
            % corresponding to the SO2 object P.

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

        
        function R = interp(obj1, obj2, s)
            %SO2.interp Interpolate between SO2 objects
            %
            % P1.interp(P2, s) is an SO2 object representing interpolation
            % between rotations represented by SO2 objects P1 and P2.  s varies from 0
            % (P1) to 1 (P2). If s is a vector (1xN) then the result will be a vector
            % of SO2 objects.
            %
            % Notes::
            % - It is an error if S is outside the interval 0 to 1.
            %
            % See also SO2.angle.
            assert(all(s>=0 & s<=1), 'RTB:SO2:interp:badarg', 's must be in the interval [0,1]');
            
            th1 = obj1.angle; th2 = obj2.angle;
            
            R = SO2( th1 + s*(th2-th1) );
        end
        
        function R = trinterp(obj, varargin)
            R = obj.interp(varargin{:});
        end
        
        function R = trinterp2(obj, varargin)
            R = obj.interp(varargin{:});
        end
        

        
        function n = new(obj, varargin)
            %SE2.new  Construct a new object of the same type
            %
            % P2 = P.new(X) creates a new object of the same type as P, by invoking the SO2 constructor on the matrix
            % X (2x2).
            %
            % P2 = P.new() as above but defines a null motion.
            %
            % Notes::
            % - Serves as a dynamic constructor.
            % - This method is polymorphic across all RTBPose derived classes, and
            %   allows easy creation of a new object of the same class as an existing
            %   one.
            %
            % See also SE3.new, SO3.new, SE2.new.
            
            n = SO2(varargin{:});
        end

    end
    methods (Static)
        % Static factory methods for constructors from exotic representations
        
        function obj = exp(s)
            %SO2.exp  Construct SO2 object from Lie algebra
            %
            % P = SO2.exp(so2) creates an SO2 object by exponentiating the se(2)
            % argument (2x2).
            obj = SO2( trexp2(s) );
        end
       
        
        function R = check(tr)
            %SO2.check  Convert to SO2
            %
            % Q = SO2.check(X) is an SO2 object where X is SO2, 2x2, SE2 or 3x3
            % homogeneous transformation matrix.
            if isa(tr, 'SO2')
                R = SO2(tr);        % is SO2 or SE2, enforce it being an SO2
            elseif SO2.isa(tr)
                R = SO2(tr);        % is 2x2
            elseif SE2.isa(tr)
                R = SO2( t2r(tr) );  % is 3x3, assume SE(2), take rotational part
            else
                error('expecting an SO2 or 2x2 matrix');
            end
        end
        
        
        function h = isa(r, dtest)
            %SO2.ISA Test if matrix is SO(2)
            %
            % SO2.ISA(T) is true (1) if the argument T is of dimension 2x2 or 2x2xN, else
            % false (0).
            %
            % SO2.ISA(T, true) as above, but also checks the validity of the rotation
            % matrix, ie. its determinant is +1.
            %
            % Notes::
            % - The first form is a fast, but incomplete, test for a transform in SE(3).
            %
            % See also SO3.ISA, SE2.ISA, SE2.ISA, ishomog2.
            d = size(r);
            if (isfloat(r) || isa(r, 'sym') ) && ndims(r) >= 2
                h =  all(d(1:2) == [2 2]);
                
                if h && nargin > 1
                    h = abs(det(r) - 1) < eps;
                end
            else
                h = false;
            end
        end
        
        function T = rand()
            %SO2.rand Construct a random SO(2) object
            %
            % SO2.rand() is an SO2 object with a uniform random orientation.
            % Random numbers are in the interval 0 to 1.
            %
            % See also RAND.
            T = SO2(rand(1,1));
        end
    end
end

