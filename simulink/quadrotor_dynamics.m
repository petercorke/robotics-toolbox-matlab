
function [sys,x0,str,ts] = quadrotor_dynamics(t,x,u,flag, quad)
    % Flyer2dynamics lovingly coded by Paul Pounds, first coded 12/4/04
    % A simulation of idealised X-4 Flyer II flight dynamics.
    % version 2.0 2005 modified to be compatible with latest version of Matlab
    % version 3.0 2006 fixed rotation matrix problem
    % version 4.0 4/2/10, fixed rotor flapping rotation matrix bug, mirroring
    
    warning off MATLAB:divideByZero
    
    % New in version 2:
    %   - Generalised rotor thrust model
    %   - Rotor flapping model
    %   - Frame aerodynamic drag model
    %   - Frame aerodynamic surfaces model
    %   - Internal motor model
    %   - Much coolage
    
    % Version 1.3
    %   - Rigid body dynamic model
    %   - Rotor gyroscopic model
    %   - External motor model
    
    %ARGUMENTS
    %   u       Reference inputs                1x4
    %   tele    Enable telemetry (1 or 0)       1x1
    %   crash   Enable crash detection (1 or 0) 1x1
    %   init    Initial conditions              1x12
    
    %INPUTS
    %   u = [N S E W]
    %   NSEW motor commands                     1x4
    
    %CONTINUOUS STATES
    %   z      Position                         3x1   (x,y,z)
    %   v      Velocity                         3x1   (xd,yd,zd)
    %   n      Attitude                         3x1   (Y,P,R)
    %   o      Angular velocity                 3x1   (wx,wy,wz)
    %   w      Rotor angular velocity           4x1
    
    %CONTINUOUS STATE MATRIX MAPPING
    %   x = [z1 z2 z3 n1 n2 n3 z1 z2 z3 o1 o2 o3 w1 w2 w3 w4]
    
    %INITIAL CONDITIONS
    z0 = [-1 0 -.15];              %   z0      Position initial conditions         1x3
    n0 = [0 0 0];               %   n0      Ang. position initial conditions    1x3
    v0 = [0 0 0];               %   v0      Velocity Initial conditions         1x3
    o0 = [0 0 0];               %   o0      Ang. velocity initial conditions    1x3
    init = [z0 n0 v0 o0];
    
    %CONTINUOUS STATE EQUATIONS
    %   z` = v
    %   v` = g*e3 - (1/m)*T*R*e3
    %   I*o` = -o X I*o + G + torq
    %   R = f(n)
    %   n` = inv(W)*o
    
    % Dispatch the flag.
    %
    
    switch flag
        case 0
            [sys,x0,str,ts]=mdlInitializeSizes(init, quad); % Initialization
        case 1
            sys = mdlDerivatives(t,x,u, quad); % Calculate derivatives
        case 3
            sys = mdlOutputs(t,x, quad); % Calculate outputs
        case { 2, 4, 9 } % Unused flags
            sys = [];
        otherwise
            error(['Unhandled flag = ',num2str(flag)]); % Error handling
    end
end % End of flyer2dynamics

%==============================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the
% S-function.
%==============================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes(init, quad)
    %
    % Call simsizes for a sizes structure, fill it in and convert it
    % to a sizes array.
    %
    sizes = simsizes;
    sizes.NumContStates  = 12;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 12;
    sizes.NumInputs      = 4;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    %
    % Initialize the initial conditions.
    x0 = init;
    %
    % str is an empty matrix.
    str = [];
    %
    % Generic timesample
    ts = [0 0];
    
    if quad.verbose
        disp(sprintf('t\t\tz1\t\tz2\t\tz3\t\tn1\t\tn2\t\tn3\t\tv1\t\tv2\t\tv3\t\to1\t\to2\t\to3\t\tw1\t\tw2\t\tw3\t\tw4\t\tu1\t\tu2\t\tu3\t\tu4'))
    end
end % End of mdlInitializeSizes.


%==============================================================
% mdlDerivatives
% Calculate the state derivatives for the next timestep
%==============================================================
%
function sys = mdlDerivatives(t,x,u, quad)
    global a1s b1s
    
    %CONSTANTS
    %Cardinal Direction Indicies
    N = 1;                      %   N       'North'                             1x1
    E = 2;                      %   S       'South'                             1x1
    S = 3;                      %   E       'East'                              1x1
    W = 4;                      %   W       'West'                              1x1
    
    
    D(:,1) = [quad.d;0;quad.h];          %   Di      Rotor hub displacements             1x3
    D(:,2) = [0;quad.d;quad.h];
    D(:,3) = [-quad.d;0;quad.h];
    D(:,4) = [0;-quad.d;quad.h];
    
    %Body-fixed frame references
    e1 = [1;0;0];               %   ei      Body fixed frame references         3x1
    e2 = [0;1;0];
    e3 = [0;0;1];
    
    %EXTRACT STATES FROM U
    w = u(1:4);
    
    %EXTRACT STATES FROM X
    z = x(1:3);   % position in {W}
    n = x(4:6);   % RPY angles {W}
    v = x(7:9);   % velocity in {W}
    o = x(10:12); % angular velocity in {W}
    
    %PREPROCESS ROTATION AND WRONSKIAN MATRICIES
    phi = n(1);    % yaw
    the = n(2);    % pitch
    psi = n(3);    % roll
    
    % rotz(phi)*roty(the)*rotx(psi)
    R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
         cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
         -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];
    
    
    %Manual Construction
    %     Q3 = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];   % RZ %Rotation mappings
    %     Q2 = [cos(the) 0 sin(the);0 1 0;-sin(the) 0 cos(the)];   % RY
    %     Q1 = [1 0 0;0 cos(psi) -sin(psi);0 sin(psi) cos(psi)];   % RX
    %     R = Q3*Q2*Q1    %Rotation matrix
    %
    %    RZ * RY * RX
    iW = [0        sin(psi)          cos(psi);             %inverted Wronskian
          0        cos(psi)*cos(the) -sin(psi)*cos(the);
          cos(the) sin(psi)*sin(the) cos(psi)*sin(the)] / cos(the);
    
    %ROTOR MODEL
    for i=[N E S W] %for each rotor
        %Relative motion
        
        Vr = cross(o,D(:,i)) + v;
        mu = sqrt(sum(Vr(1:2).^2)) / (abs(w(i))*quad.r);  %Magnitude of mu, planar components
        lc = Vr(3) / (abs(w(i))*quad.r);   %Non-dimensionalised normal inflow
        li = mu; %Non-dimensionalised induced velocity approximation
        alphas = atan2(lc,mu);
        j = atan2(Vr(2),Vr(1));  %Sideslip azimuth relative to e1 (zero over nose)
        J = [cos(j) -sin(j);
            sin(j) cos(j)];  %BBF > mu sideslip rotation matrix
        
        %Flapping
        beta = [((8/3*quad.theta0 + 2*quad.theta1)*mu - 2*(lc)*mu)/(1-mu^2/2); %Longitudinal flapping
            0;];%sign(w) * (4/3)*((Ct/sigma)*(2*mu*gamma/3/a)/(1+3*e/2/r) + li)/(1+mu^2/2)]; %Lattitudinal flapping (note sign)
        beta = J'*beta;  %Rotate the beta flapping angles to longitudinal and lateral coordinates.
        a1s(i) = beta(1) - 16/quad.gamma/abs(w(i)) * o(2);
        b1s(i) = beta(2) - 16/quad.gamma/abs(w(i)) * o(1);
        
        %Forces and torques
        T(:,i) = quad.Ct*quad.rho*quad.A*quad.r^2*w(i)^2 * [-cos(b1s(i))*sin(a1s(i)); sin(b1s(i));-cos(a1s(i))*cos(b1s(i))];   %Rotor thrust, linearised angle approximations
        Q(:,i) = -quad.Cq*quad.rho*quad.A*quad.r^3*w(i)*abs(w(i)) * e3;     %Rotor drag torque - note that this preserves w(i) direction sign
        tau(:,i) = cross(T(:,i),D(:,i));    %Torque due to rotor thrust
    end
    
    %RIGID BODY DYNAMIC MODEL
    dz = v;
    dn = iW*o;
    
    dv = quad.g*e3 + R*(1/quad.M)*sum(T,2);
    do = inv(quad.J)*(cross(-o,quad.J*o) + sum(tau,2) + sum(Q,2)); %row sum of torques
    sys = [dz;dn;dv;do];   %This is the state derivative vector
end % End of mdlDerivatives.


%==============================================================
% mdlOutputs
% Calculate the output vector for this timestep
%==============================================================
%
function sys = mdlOutputs(t,x, quad)
    %CRASH DETECTION
    if x(3)>0
        x(3) = 0;
        if x(6) > 0
            x(6) = 0;
        end
    end
    %if (x(3)>0)&(crash)
    %    error('CRASH!')
    %end
    
    %TELEMETRY
    if quad.verbose
        disp(sprintf('%0.3f\t',t,x))
    end
    
    % compute output vector as a function of state vector
    %   z      Position                         3x1   (x,y,z)
    %   v      Velocity                         3x1   (xd,yd,zd)
    %   n      Attitude                         3x1   (Y,P,R)
    %   o      Angular velocity                 3x1   (Yd,Pd,Rd)
    
    n = x(4:6);   % RPY angles
    phi = n(1);    % yaw
    the = n(2);    % pitch
    psi = n(3);    % roll
    
    
    % rotz(phi)*roty(the)*rotx(psi)
    R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
         cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
         -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];
    
    iW = [0        sin(psi)          cos(psi);             %inverted Wronskian
          0        cos(psi)*cos(the) -sin(psi)*cos(the);
          cos(the) sin(psi)*sin(the) cos(psi)*sin(the)] / cos(the);
    
    % return velocity in the body frame
    sys = [ x(1:6);
            inv(R)*x(7:9);   % translational velocity mapped to body frame
            iW*x(10:12)];    % RPY rates mapped to body frame
    %sys = [x(1:6); iW*x(7:9);  iW*x(10:12)];
    %sys = x;
end
% End of mdlOutputs.
