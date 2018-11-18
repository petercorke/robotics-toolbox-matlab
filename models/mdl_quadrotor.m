%MDL_QUADCOPTER Dynamic parameters for a quadrotor.
%
% MDL_QUADCOPTER is a script creates the workspace variable quad which
% describes the dynamic characterstics of a quadrotor flying robot.
%
% Properties::
%
% This is a structure with the following elements:
%
% nrotors   Number of rotors (1x1)
% J         Flyer rotational inertia matrix (3x3)
% h         Height of rotors above CoG (1x1)
% d         Length of flyer arms (1x1)
% nb        Number of blades per rotor (1x1)
% r         Rotor radius (1x1)
% c         Blade chord (1x1)
% e         Flapping hinge offset (1x1)
% Mb        Rotor blade mass (1x1)
% Mc        Estimated hub clamp mass (1x1)
% ec        Blade root clamp displacement (1x1)
% Ib        Rotor blade rotational inertia (1x1)
% Ic        Estimated root clamp inertia (1x1)
% mb        Static blade moment (1x1)
% Ir        Total rotor inertia (1x1)
% Ct        Non-dim. thrust coefficient (1x1)
% Cq        Non-dim. torque coefficient (1x1)
% sigma     Rotor solidity ratio (1x1)
% thetat    Blade tip angle (1x1)
% theta0    Blade root angle (1x1)
% theta1    Blade twist angle (1x1)
% theta75   3/4 blade angle (1x1)
% thetai    Blade ideal root approximation (1x1)
% a         Lift slope gradient (1x1)
% A         Rotor disc area (1x1)
% gamma     Lock number (1x1)
%
%
% Notes::
% - SI units are used.
%
% References::
% - Design, Construction and Control of a Large Quadrotor micro air vehicle.
%   P.Pounds, PhD thesis, 
%   Australian National University, 2007.
%   http://www.eng.yale.edu/pep5/P_Pounds_Thesis_2008.pdf
% - This is a heavy lift quadrotor
%
% See also sl_quadrotor.



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

quadrotor.nrotors = 4;                %   4 rotors
quadrotor.g = 9.81;                   %   g       Gravity                             1x1
quadrotor.rho = 1.184;                %   rho     Density of air                      1x1
quadrotor.muv = 1.5e-5;               %   muv     Viscosity of air                    1x1

% Airframe
quadrotor.M = 4;                      %   M       Mass                                1x1
% Ixx = 0.082;
% Iyy = 0.082;
% Izz = 0.149;%0.160;
quadrotor.J = diag([0.082 0.082 0.149]);    %   I       Flyer rotational inertia matrix     3x3

quadrotor.h = -0.007;                 %   h       Height of rotors above CoG          1x1
quadrotor.d = 0.315;                  %   d       Length of flyer arms                1x1

%Rotor
quadrotor.nb = 2;                      %   b       Number of blades per rotor          1x1
quadrotor.r = 0.165;                  %   r       Rotor radius                        1x1

quadrotor.c = 0.018;                  %   c       Blade chord                         1x1

quadrotor.e = 0.0;                    %   e       Flapping hinge offset               1x1
quadrotor.Mb = 0.005;                 %   Mb      Rotor blade mass                    1x1
quadrotor.Mc = 0.010;                 %   Mc      Estimated hub clamp mass            1x1
quadrotor.ec = 0.004;                 %   ec      Blade root clamp displacement       1x1
quadrotor.Ib = quadrotor.Mb*(quadrotor.r-quadrotor.ec)^2/4 ;        %   Ib      Rotor blade rotational inertia      1x1
quadrotor.Ic = quadrotor.Mc*(quadrotor.ec)^2/4;           %   Ic      Estimated root clamp inertia        1x1
quadrotor.mb = quadrotor.g*(quadrotor.Mc*quadrotor.ec/2+quadrotor.Mb*quadrotor.r/2);    %   mb      Static blade moment                 1x1
quadrotor.Ir = quadrotor.nb*(quadrotor.Ib+quadrotor.Ic);             %   Ir      Total rotor inertia                 1x1

quadrotor.Ct = 0.0048;                %   Ct      Non-dim. thrust coefficient         1x1
quadrotor.Cq = quadrotor.Ct*sqrt(quadrotor.Ct/2);         %   Cq      Non-dim. torque coefficient         1x1

quadrotor.sigma = quadrotor.c*quadrotor.nb/(pi*quadrotor.r);         %   sigma   Rotor solidity ratio                1x1
quadrotor.thetat = 6.8*(pi/180);      %   thetat  Blade tip angle                     1x1
quadrotor.theta0 = 14.6*(pi/180);     %   theta0  Blade root angle                    1x1
quadrotor.theta1 = quadrotor.thetat - quadrotor.theta0;   %   theta1  Blade twist angle                   1x1
quadrotor.theta75 = quadrotor.theta0 + 0.75*quadrotor.theta1;%   theta76 3/4 blade angle                     1x1
quadrotor.thetai = quadrotor.thetat*(quadrotor.r/quadrotor.e);      %   thetai  Blade ideal root approximation      1x1
quadrotor.a = 5.5;                    %   a       Lift slope gradient                 1x1

% derived constants
quadrotor.A = pi*quadrotor.r^2;                 %   A       Rotor disc area                     1x1
quadrotor.gamma = quadrotor.rho*quadrotor.a*quadrotor.c*quadrotor.r^4/(quadrotor.Ib+quadrotor.Ic);%   gamma   Lock number                         1x1

quadrotor.b = quadrotor.Ct*quadrotor.rho*quadrotor.A*quadrotor.r^2; % T = b w^2
quadrotor.k = quadrotor.Cq*quadrotor.rho*quadrotor.A*quadrotor.r^3; % Q = k w^2

quadrotor.verbose = false;

