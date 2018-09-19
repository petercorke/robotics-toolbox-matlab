%SE3 SE(3) homogeneous transformation class
%
% This subclasss of SE3 < SO3 < RTBPose is an object that represents an SE(3)
% rigid-body motion
%
% T = SE3() is an SE(3) homogeneous transformation (4x4) representing zero
% translation and rotation.
%
% T = SE3(X,Y,Z) as above represents a pure translation.
%
% T = SE3.Rx(THETA) as above represents a pure rotation about the x-axis.
%
%
% Constructor methods::
%  SE3              general constructor
%  SE3.exp          exponentiate an se(3) matrix                         
%  SE3.angvec       rotation about vector
%  SE3.eul          rotation defined by Euler angles
%  SE3.oa           rotation defined by o- and a-vectors
%  SE3.rpy          rotation defined by roll-pitch-yaw angles
%  SE3.Rx           rotation about x-axis
%  SE3.Ry           rotation about y-axis
%  SE3.Rz           rotation about z-axis
%  SE3.rand         random transformation
%  new              new SE3 object
%
% Information and test methods::
%  dim*             returns 4
%  isSE*            returns true
%  issym*           true if rotation matrix has symbolic elements
%  isidentity       true for null motion
%  SE3.isa          check if matrix is SO2
%
% Display and print methods::
%  plot*            graphically display coordinate frame for pose
%  animate*         graphically animate coordinate frame for pose
%  print*           print the pose in single line format
%  display*         print the pose in human readable matrix form
%  char*            convert to human readable matrix as a string
%
% Operation methods::
%  det              determinant of matrix component
%  eig              eigenvalues of matrix component
%  log              logarithm of rotation matrixr>=0 && r<=1ub
%  inv              inverse
%  simplify*        apply symbolic simplication to all elements
%  Ad               adjoint matrix (6x6)
%  increment        update pose based on incremental motion
%  interp           interpolate poses
%  velxform         compute velocity transformation
%  interp           interpolate between poses
%  ctraj            Cartesian motion
%
% Conversion methods::
%  SE3.check        convert object or matrix to SE3 object
%  double           convert to rotation matrix
%  R                return rotation matrix
%  SO3              return rotation part as an SO3 object 
%  T                convert to homogeneous transformation matrix
%  UnitQuaternion   convert to UnitQuaternion object
%  toangvec         convert to rotation about vector form
%  toeul            convert to Euler angles
%  torpy            convert to roll-pitch-yaw angles
%  t                translation column vector
%  tv               translation column vector for vector of SE3
%
% Compatibility methods::
%  homtrans         apply to vector
%  isrot*           returns false
%  ishomog*         returns true
%  tr2rt*       convert to rotation matrix and translation vector
%  t2r*         convert to rotation matrix
%  trprint*         print single line representation
%  trplot*          plot coordinate frame
%  tranimate*       animate coordinate frame
%  tr2eul           convert to Euler angles
%  tr2rpy           convert to roll-pitch-yaw angles
%  trnorm           normalize the rotation matrix
%  transl           return translation as a row vector  
%
% * means inherited from RTBPose
%
% Operators::
%  +               elementwise addition, result is a matrix
%  -               elementwise subtraction, result is a matrix
%  *               multiplication within group, also group x vector
%  .*              multiplication within group followed by normalization
%  /               multiply by inverse
%  ./              multiply by inverse followed by normalization
%  ==          test equality
%  ~=          test inequality
%
% Properties::
%  n              normal (x) vector
%  o              orientation (y) vector
%  a              approach (z) vector
%  t              translation vector
%
% For single SE3 objects only, for a vector of SE3 objects use the
% equivalent methods
% t       translation as a 3x1 vector (read/write)
% R       rotation as a 3x3 matrix (read/write)
%
% Methods::
% tv      return translations as a 3xN vector
%
% Notes::
% - The properies R, t are implemented as MATLAB dependent properties.
%   When applied to a vector of SE3 object the result is a comma-separated
%   list which can be converted to a matrix by enclosing it in square
%   brackets, eg [T.t] or more conveniently using the method T.transl
%
% See also SO3, SE2, RTBPose.

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
        %         T
        %R
    end
    
    methods
        
        function obj = SE3(varargin)
            %SE3.SE3 Create an SE(3) object
            %
            % Constructs an SE(3) pose object that contains a 4x4 homogeneous transformation matrix.
            %
            % T = SE3() is a null relative motion
            %
            % T = SE3(X, Y, Z) is an object representing pure translation defined by X,
            % Y and Z.
            %
            % T = SE3(XYZ) is an object representing pure translation defined by XYZ
            % (3x1).  If XYZ (Nx3) returns an array of SE3 objects, corresponding to
            % the rows of XYZ
            %
            % T = SE3(R, XYZ) is an object representing rotation defined by the
            % orthonormal rotation matrix R (3x3) and position given by XYZ (3x1)
            %
            % T = SE3(T) is an object representing translation and rotation defined by
            % the homogeneous transformation matrix T (3x3).  If T (3x3xN) returns an array of SE3 objects, corresponding to
            % the third index of T
            %
            % T = SE3(T) is an object representing translation and rotation defined by
            % the SE3 object T, effectively cloning the object. If T (Nx1) returns an array of SE3 objects, corresponding to
            % the index of T
            %
            % Options::
            % 'deg'         Angle is specified in degrees
            %
            % Notes::
            % - Arguments can be symbolic
            
            obj.data = eye(4,4);
            args = varargin;
            
            % if any of the arguments is symbolic the result will be symbolic
            if any( cellfun(@(x) isa(x, 'sym'), args) )
                obj.data = sym(obj.data);
            end
            
            switch length(args)
                case 0
                    % ()
                    obj.data = eye(4,4);
                    return;
                    
                case 1
                    a = args{1};
                    if isvec(a, 3)
                        % (t)
                        obj.t = a(:);
                    elseif SE3.isa(a)
                        % (SE3)
                        for i=1:size(a, 3)
                            obj(i).data = a(:,:,i);
                        end
                    elseif isa(a, 'SE3')
                        % (T)
                        for i=1:length(a)
                            obj(i).data = a(i).data;
                        end
                        
                    elseif numcols(a) == 3
                        % SE3( xyz )
                        for i=1:length(a)
                            %obj(i).data = SE3(a(i,:));
                            obj(i).data(1:3,4) = a(i,:)';
                        end
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                    
                case 2
                    a = args{1}; b = args{2};
                    
                    if (isrot(a) || SO3.isa(a)) && isvec(b,3)
                        % (R, t)
                        if isrot(a)
                            obj.data(1:3,1:3) = a;
                        else
                            obj.R(1:3,1:3) = a.R;
                        end
                        obj.data(1:3,4) = b(:);
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                    
                case 3
                    a = args{1}; b = args{2}; c = args{3};
                    
                    obj.data(1:3,4) = [a; b; c];
                    
                otherwise
                    error('RTB:SE3:badarg', 'too many arguments');
                    
            end
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  GET AND SET
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function o = get.t(obj)
            o = obj.data(1:3,4);
        end
        
        function o = set.t(obj, t)
            %SE3.t Get translation vector
            %
            % T = P.t is the translational part of SE3 object as a 3-element column
            % vector.
            %
            % Notes::
            % - If applied to  a vector will return a comma-separated list, use
            %   .transl() instead.
            %
            % See also SE3.transl, transl.
            
            % TODO CHECKING
            obj.data(1:3,4) = t(:);
            o = obj;
        end
        
        
        function [t1,t2,t3] = transl(obj)
            %SE3.transl Get translation vector
            %
            % T = P.transl() is the translational part of SE3 object as a 3-element row
            % vector.  If P is a vector (1xN) then
            %  the rows of T (Mx3) are the translational component of the
            % corresponding pose in the sequence.
            %
            % [X,Y,Z] = P.transl() as above but the translational part is returned as
            % three components.  If P is a vector (1xN) then X,Y,Z (1xN) are the
            % translational components of the corresponding pose in the sequence.
            %
            % Notes::
            % - The .t method only works for a single pose object, on  a vector it
            %   returns a comma-separated list.
            %
            % See also SE3.t, transl.
            
            if nargout == 1 || nargout == 0
                t1 = [obj.t]';
            else
                t = obj.t;
                t1 = t(1);
                t2 = t(2);
                t3 = t(3);
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
            TT = double(obj);
        end
              
        
        % set.R
        
        % handle assignment SE3 = SE3, SE3=4x4, can'toverload equals
        
        function t = tv(obj)
            %SE.tv Return translation for a vector of SE3 objects
            %
            % P.tv is a column vector (3x1) representing the translational part of the
            % SE3 pose object P.  If P is a vector of SE3 objects (Nx1) then the result
            % is a matrix (3xN) with columns corresponding to the elements of P.
            %
            % See also SE3.t.
            
            %t = zeros(3,length(obj)); FAILS FOR SYMBOLIC CASE
            for i=1:length(obj)
                t(:,i) = obj(i).data(1:3,4);
            end
        end
        

        function v = isidentity(obj)
            %SE3.velxform  Apply incremental motion to an SE(3) pose
            %
            % P.isidentity() is true of the SE3 object P corresponds to null motion,
            % that is, its homogeneous transformation matrix is identity.
            
            v = all(all(obj.T == eye(4,4)));
        end
        
        function T = increment(obj, v)
            %SE3.increment  Apply incremental motion to an SE3 pose
            %
            % P1 = P.increment(d) is an SE3 pose object formed by applying the
            % incremental motion vector d (1x6) in the frame associated with SE3 pose
            % P.
            %
            % See also SE3.todelta, DELTA2TR, TR2DELTA.
            T = obj .* SE3(delta2tr(v));
        end
        
        function J = velxform(obj, varargin)
            %SE3.velxform  Velocity transformation
            %
            % Transform velocity between frames.  A is the world frame, B is the body
            % frame and C is another frame attached to the body.  PAB is the pose of
            % the body frame with respect to the world frame, PCB is the pose of the
            % body frame with respect to frame C.
            %
            % J = PAB.velxform() is a 6x6 Jacobian matrix that maps velocity from frame
            % B to frame A.
            %
            % J = PCB.velxform('samebody') is a 6x6 Jacobian matrix that maps velocity
            % from frame C to frame B.  This is also the adjoint of PCB.
            
            opt.samebody = false;
            
            opt = tb_optparse(opt, varargin);
            
            R = obj.R;
            
            if opt.samebody
                J = [
                        R           skew(obj.t)*R
                        zeros(3,3)  R
                    ];
            else
                J = [
                        R            zeros(3,3)
                        zeros(3,3)   R
                    ];
            end
        end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  STANDARD SE(3) MATRIX OPERATIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        function it = inv(obj)
            %SE3.inv  Inverse of SE3 object
            %
            % Q = inv(P) is the inverse of the SE3 object P.  P*Q will be the identity
            % matrix.
            %
            % Notes::
            % - This is formed explicitly, no matrix inverse required.
            
            it = SE3( obj.R', -obj.R'*obj.t);
        end
        
        function Ti = interp(obj1, varargin)
            %SE3.interp Interpolate SE3 poses
            %
            % P1.interp(P2, s) is an SE3 object representing an interpolation
            % between poses represented by SE3 objects P1 and P2.  s varies from 0
            % (P1) to 1 (P2).  If s is a vector (1xN) then the result will be a vector
            % of SO3 objects.
            %
            % P1.interp(P2,N) as above but returns a vector (1xN) of SE3 objects
            % interpolated between P1 and P2 in N steps.
            %
            % Notes::
            % - The rotational interpolation (slerp) can be interpretted
            %   as interpolation along a great circle arc on a sphere.
            % - It is an error if S is outside the interval 0 to 1.
            %
            % See also TRINTERP, UnitQuaternion.
            
            if isa(varargin{1}, 'SE3')
                % interp(SE3, SE3, s)  interpolate between given values
                obj2 = varargin{1};
                varargin = varargin(2:end);
                Ti = SE3( trinterp(obj1.T, obj2.T, varargin{:}) );
            else
                % interp(SE3, s)  interpolate between null and given value
                Ti = SE3( trinterp(obj1.T, varargin{:}) );
            end
        end
        
        function traj = ctraj(T0, T1, t)
            %SE3.ctraj Cartesian trajectory between two poses
            %
            % TC = T0.ctraj(T1, N) is a Cartesian trajectory defined by a vector of SE3
            % objects (1xN) from pose T0 to T1, both described by SE3 objects.  There
            % are N points on the trajectory that follow a trapezoidal velocity profile
            % along the trajectory.
            % 
            % TC = CTRAJ(T0, T1, S) as above but the elements of S (Nx1) specify the 
            % fractional distance  along the path, and these values are in the range [0 1].
            % The i'th point corresponds to a distance S(i) along the path.
            %
            % Notes::
            % - In the second case S could be generated by a scalar trajectory generator
            %   such as TPOLY or LSPB (default).
            % - Orientation interpolation is performed using quaternion interpolation.
            %
            % Reference::
            % Robotics, Vision & Control, Sec 3.1.5,
            % Peter Corke, Springer 2011
            %
            % See also LSPB, MSTRAJ, TRINTERP, CTRAJ, UnitQuaternion.interp.

            assert(isa(T1, 'SE3'), 'second transform must be SE3');
            
            % distance along path is a smooth function of time
            if isscalar(t)
                s = lspb(0, 1, t);
            else
                s = t(:);
            end

            for i=1:length(s)    
                traj(i) = T0.interp(T1, s(i));
            end
        end
        
        function m = log(obj)
            %SE3.log  Lie algebra
            %
            % se3 = P.log() is the Lie algebra expressed as an augmented skew-symmetric
            % matrix (4x4) corresponding to the SE3 object P.
            %
            % See also SE3.logs, SE3.Twist, trlog, logm.
            m = trlog(obj.T);
        end
        
        function m = logs(obj)
            %SE3.log  Lie algebra
            %
            % se3 = P.log() is the Lie algebra expressed as vector (1x6)
            % corresponding to the SE2 object P.  The vector comprises the
            % translational elements followed by the unique elements of the
            % skew-symmetric rotation submatrix.
            %
            % See also SE3.log, SE3.Twist, trlog, logm.
            
            m = trlog(obj.T);
            m = [m(1:3,4); vex(m(1:3,1:3))]';
        end
        
        function tw = Twist(obj)
            %SE3.Twist  Convert to Twist object
            %
            % TW = P.Twist() is the equivalent Twist object.  The elements of the twist are the unique
            % elements of the Lie algebra of the SE3 object P.
            %
            % See also SE3.logs, Twist.
            tw = Twist( obj.log );
        end
        
        
        function m = Ad(obj)
            %SE3.Ad  Adjoint matrix
            %
            % A = S.Ad() is the adjoint matrix (6x6) corresponding to the SE(3) value
            % S.
            %
            % See also Twist.ad.
            m = [
                    obj.R       skew(obj.t)*obj.R
                    zeros(3,3)  obj.R
                ];
        end
        
        %% ad function
        %         function m = Ad(obj)
        %             m = obj.R;
        %         end
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  Rotation conversion wrappers
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        function out = toeul(obj, varargin)
            %SE3.toeul Convert  to Euler angles
            %
            % EUL = P.toeul(OPTIONS) are the ZYZ Euler angles (1x3) corresponding to
            % the rotational part of the SE3 object P. The 3 angles EUL=[PHI,THETA,PSI]
            % correspond to sequential rotations about the Z, Y and Z axes
            % respectively.
            %
            % If P is a vector (1xN) then each row of EUL corresponds to an element of
            % the vector.
            %
            % Options::
            %  'deg'      Compute angles in degrees (radians default)
            %  'flip'     Choose first Euler angle to be in quadrant 2 or 3.
            %
            % Notes::
            % - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
            %   set to zero and PSI is the sum (PHI+PSI).
            %
            % See also SO3.toeul, SE3.torpy, EUL2TR, TR2RPY.
            out = obj.SO3.toeul(varargin{:});
        end
        
        function out = torpy(obj, varargin)
            %SE3.RPY Convert to roll-pitch-yaw angles
            %
            % RPY = P.torpy(options) are the roll-pitch-yaw angles (1x3) corresponding
            % to the rotational part of the SE3 object P. The 3 angles RPY=[R,P,Y]
            % correspond to sequential rotations about the Z, Y and X axes
            % respectively.
            %
            % If P is a vector (1xN) then each row of RPY corresponds to an element of
            % the vector.
            %
            % Options::
            %  'deg'   Compute angles in degrees (radians default)
            %  'xyz'   Return solution for sequential rotations about X, Y, Z axes
            %  'yxz'   Return solution for sequential rotations about Y, X, Z axes
            %
            % Notes::
            % - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
            %   set to zero and Y is the sum (R+Y).
            %
            % See also SE3.torpy, SE3.toeul, rpy2tr, tr2eul.
            out = obj.SO3.torpy(varargin{:});
        end
        
        function [a,b] = toangvec(obj, varargin)
            %SE3.toangvec Convert to angle-vector form
            %
            % [THETA,V] = P.toangvec(OPTIONS) is rotation expressed in terms of an
            % angle THETA (1x1) about the axis V (1x3) equivalent to the rotational
            % part of the SE3 object P.
            %
            % If P is a vector (1xN) then THETA (Kx1) is a vector of angles for
            % corresponding elements of the vector and V (Kx3) are the corresponding
            % axes, one per row.
            %
            % Options::
            % 'deg'   Return angle in degrees
            %
            % Notes::
            % - If no output arguments are specified the result is displayed.
            %
            % See also ANGVEC2R, ANGVEC2TR, TRLOG.
            if nargout == 1
                a = obj.SO3.toangvec(varargin{:});
            else
                [a,b] = obj.SO3.toangvec(varargin{:});
            end
        end
        
        function d = todelta(P1, P2)
            %SE3.todelta Convert SE(3) object to differential motion vector
            %
            % D = SE3.todelta(P0, P1) is the (6x1) differential motion vector (dx, dy,
            % dz, dRx, dRy, dRz) corresponding to infinitessimal motion (in the P0
            % frame) from SE(3) pose P0 to P1. .
            %
            % D = SE3.todelta(P) as above but the motion is with respect to the world frame.
            %
            % Notes::
            % - D is only an approximation to the motion, and assumes
            %   that P0 ~ P1 or P ~ eye(4,4).
            % - can be considered as an approximation to the effect of spatial velocity over a
            %   a time interval, average spatial velocity multiplied by time.
            %
            % See also SE3.increment, TR2DELTA, DELTA2TR.
            
            if nargin == 1
                d = tr2delta(P1.T);
            elseif nargin == 2
                d = tr2delta(P1.T, P2.T);
            end
        end
        
        % conversion methods
        
        function s = SO3(obj)
            %SE3.SO3 Convert rotational component to SO3 object
            %
            % P.SO3 is an SO3 object representing the rotational component of the SE3
            % pose P.  If P is a vector (Nx1) then the result is a vector (Nx1).
            for i=1:length(obj)
                s(i) = SO3();
                s(i).data= obj(i).R;
            end
        end
        
        function n = new(obj, varargin)
            %SE3.new  Construct a new object of the same type
            %
            % P2 = P.new(X) creates a new object of the same type as P, by invoking the SE3 constructor on the matrix
            % X (4x4).
            %
            % P2 = P.new() as above but defines a null motion.
            %
            % Notes::
            % - Serves as a dynamic constructor.
            % - This method is polymorphic across all RTBPose derived classes, and
            %   allows easy creation of a new object of the same class as an existing
            %   one.
            %
            % See also SO3.new, SO2.new, SE2.new.

            n = SE3(varargin{:});
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  VANILLA RTB FUNCTION COMPATIBILITY
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function Vt = homtrans(P, V)
            %SE3.homtrans  Apply transformation to points
            %
            % P.homtrans(V) applies SE3 pose object P to the points stored columnwise in
            % V (3xN) and returns transformed points (3xN).
            %
            % Notes::
            % - P is an SE3 object defining the pose of {A} with respect to {B}.
            % - The points are defined with respect to frame {A} and are transformed to be
            %   with respect to frame {B}.
            % - Equivalent to P*V using overloaded SE3 operators.
            %
            % See also RTBPose.mtimes, HOMTRANS.
            Vt = P*V;
        end
        
        
        function v = ishomog(obj)
            v = true;
        end
        
        function v = ishomog2(obj)
            v = false;
        end
        
        function v = isvec(obj, n)
            v = false;
        end
        
    end
    
    methods (Static)
        
        
        % Static factory methods for constructors from standard representations
        
        function obj = Rx(varargin)
            %SE3.Rx Rotation about X axis
            %
            % P = SE3.Rx(THETA) is an SE3 object representing a rotation of THETA
            % radians about the x-axis.  If the THETA is a vector (1xN) then P will be
            % a vector (1xN) of corresponding SE3 objects.
            %
            % P = SE3.Rx(THETA, 'deg') as above but THETA is in degrees.
            %
            % See also SE3.Ry, SE3.Rz, rotx.
            obj = SE3( SO3.Rx(varargin{:}) );
        end
        function obj = Ry(varargin)
            %SE3.Ry Rotation about Y axis
            %
            % P = SE3.Ry(THETA) is an SE3 object representing a rotation of THETA
            % radians about the y-axis.  If the THETA is a vector (1xN) then P will be
            % a vector (1xN) of corresponding SE3 objects.
            %
            % P = SE3.Ry(THETA, 'deg') as above but THETA is in degrees.
            %
            % See also SE3.Ry, SE3.Rz, rotx.
            obj = SE3( SO3.Ry(varargin{:}) );
        end
        function obj = Rz(varargin)
            %SE3.Rz Rotation about Z axis
            %
            % P = SE3.Rz(THETA) is an SE3 object representing a rotation of THETA
            % radians about the z-axis.  If the THETA is a vector (1xN) then P will be
            % a vector (1xN) of corresponding SE3 objects.
            %
            % P = SE3.Rz(THETA, 'deg') as above but THETA is in degrees.
            %
            % See also SE3.Ry, SE3.Rz, rotx.
            obj = SE3( SO3.Rz(varargin{:}) );
        end
        
        function obj = oa(varargin)
            %SE3.oa Construct an SE(3) object from orientation and approach vectors 
            %
            % P = SE3.oa(O, A) is an SE3 object for the specified
            % orientation and approach vectors (3x1) formed from 3 vectors such that R
            % = [N O A] and N = O x A, with zero translation.
            %
            % Notes::
            % - The rotation submatrix is guaranteed to be orthonormal so long as O and A 
            %   are not parallel.
            % - The vectors O and A are parallel to the Y- and Z-axes of the coordinate
            %   frame.
            %
            % References::
            % - Robot manipulators: mathematis, programming and control
            %   Richard Paul, MIT Press, 1981.
            %
            % See also RPY2R, EUL2R, OA2TR, SO3.oa.
            obj = SE3( SO3.oa(varargin{:}) );
        end
        
        function obj = angvec(varargin)
            %SE3.angvec Construct an SE(3) object from angle and axis vector
            % 
            % R = SE3.angvec(THETA, V) is an orthonormal rotation matrix (3x3)
            % equivalent to a rotation of THETA about the vector V.
            %
            % Notes::
            % - If THETA == 0 then return identity matrix.
            % - If THETA ~= 0 then V must have a finite length.
            %
            % See also SO3.angvec, eul2r, rpy2r, tr2angvec.
            obj = SE3( SO3.angvec(varargin{:}) );
        end
        
        function obj = rpy(varargin)
            %SE3.rpy Construct an SE(3) object from roll-pitch-yaw angles
            %
            % P = SE3.rpy(ROLL, PITCH, YAW, OPTIONS) is an SE3 object equivalent to the
            % specified roll, pitch, yaw angles angles with zero translation. These correspond to rotations
            % about the Z, Y, X axes respectively. If ROLL, PITCH, YAW are column
            % vectors (Nx1) then they are assumed to represent a trajectory then P is a
            % vector (1xN) of SE3 objects.
            %
            % P = SE3.rpy(RPY, OPTIONS) as above but the roll, pitch, yaw angles angles
            % angles are taken from consecutive columns of the passed matrix RPY =
            % [ROLL, PITCH, YAW].  If RPY is a matrix (Nx3) then they are assumed to
            % represent a trajectory and P is a vector (1xN) of SE3 objects.
            %
            % Options::
            %  'deg'   Compute angles in degrees (radians default)
            %  'xyz'   Rotations about X, Y, Z axes (for a robot gripper)
            %  'yxz'   Rotations about Y, X, Z axes (for a camera)
            %
            % See also SO3.rpy, SE3.eul, TR2RPY, EUL2TR.
            obj = SE3( SO3.rpy(varargin{:}) );
        end
        
        function obj = eul(varargin)
            %SE3.eul Construct an SE(3) object from Euler angles
            %
            % P = SE3.eul(PHI, THETA, PSI, OPTIONS) is an SE3 object equivalent to the
            % specified Euler angles with zero translation.  These correspond to rotations about the Z, Y, Z
            % axes respectively. If PHI, THETA, PSI are column vectors (Nx1) then they
            % are assumed to represent a trajectory then P is a vector (1xN) of SE3 objects.
            %
            % P = SE3.eul2R(EUL, OPTIONS) as above but the Euler angles are taken from
            % consecutive columns of the passed matrix EUL = [PHI THETA PSI].  If EUL
            % is a matrix (Nx3) then they are assumed to represent a trajectory then P
            % is a vector (1xN) of SE3 objects.
            %
            % Options::
            %  'deg'      Compute angles in degrees (radians default)
            %
            % Note::
            % - The vectors PHI, THETA, PSI must be of the same length.
            %
            % See also SO3.eul, SE3.rpy, EUL2TR, RPY2TR, TR2EUL.
            obj = SE3( SO3.eul(varargin{:}) );
        end
        
        function T = rand()
            %SE3.rand Construct a random SE(3) object
            %
            % SE3.rand() is an SE3 object with a uniform random translation and a
            % uniform random RPY/ZYX orientation.  Random numbers are in the interval 0 to
            % 1.
            %
            % See also RAND.
            T = SE3( rand(3,1) ) * SE3(SO3.rand);
        end
        
        function obj = delta(d)
            %SE3.delta SE3 object from differential motion vector
            %
            % T = SE3.delta(D) is an SE3 pose object representing differential
            % translation and rotation. The vector D=(dx, dy, dz, dRx, dRy, dRz)
            % represents an infinitessimal motion, and is an approximation to the spatial
            % velocity multiplied by time.
            %
            % See also SE3.todelta, SE3.increment, TR2DELTA.
            
            assert(isvec(d,6), 'RTB:SE3:badarg', 'delta is a 6-vector');
            obj = SE3( delta2tr(d));
        end
        
        % Static factory methods for constructors from exotic representations
        function obj = exp(s)
            %SE3.exp SE3 object from se(3)
            %
            %  SE3.exp(SIGMA) is the SE3 rigid-body motion given by the se(3) element SIGMA (4x4).
            %
            %  SE3.exp(exp(S) as above, but the se(3) value is expressed as a twist vector (6x1).
            %
            %  SE3.exp(SIGMA, THETA) as above, but the motion is given by SIGMA*THETA
            %  where SIGMA is an se(3) element (4x4) whose rotation part has a unit norm.
            %
            % Notes::
            % - wraps trexp.
            %
            % See also trexp.
            obj = SE3( trexp(s) );
        end
        
        %         function m = ad(s)
        %             %m = [skew(s(1:3)) skew(s(4:6));  skew(s(4:6)) zeros(3,3)];
        %             m = [skew(s(4:6)) skew(s(1:3)) ;  zeros(3,3) skew(s(4:6)) ];
        %         end
        

        function T = check(tr)
            %SE3.check  Convert to SE3
            %
            % Q = SE3.check(X) is an SE3 object where X is SE3 object or 4x4
            % homogeneous transformation matrix.
            if isa(tr, 'SE3')
                T = tr;
            elseif SE3.isa(tr)
                T = SE3(tr);
            else
                error('expecting an SE3 or 4x4 matrix');
            end
        end
        
        function h = isa(tr, rtest)
            %SE3.ISA Test if a homogeneous transformation
            %
            % SE3.ISA(T) is true (1) if the argument T is of dimension 4x4 or 4x4xN, else
            % false (0).
            %
            % SE3.ISA(T, 'valid') as above, but also checks the validity of the rotation
            % sub-matrix.
            %
            % Notes::
            % - The first form is a fast, but incomplete, test for a transform in SE(3).
            %
            % See also SO3.ISA, SE2.ISA, SO2.ISA.
            d = size(tr);
            if ndims(tr) >= 2
                h =  all(d(1:2) == [4 4]);
                
                if h && nargin > 1
                    h = SO3.isa( tr(1:3,1:3) );
                end
            else
                h = false;
            end
        end
       
        
    end
end
