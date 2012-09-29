%MDL_BALL Create model of a ball manipulator
%
% MDL_BALL  creates the workspace variable ball which describes the 
% kinematic characteristics of a serial link manipulator that folds into
% a ball shape.  By default has 50 joints.
%
% MDL_BALL(N) as above but creates a manipulator with N joints.
%
% Also define the workspace vectors:
%   q  joint angle vector for default ball configuration
%
% Reference::
% - "A divide and conquer articulated-body algorithm for parallel O(log(n))
%   calculation of rigid body dynamics, Part 2",
%   Int. J. Robotics Research, 18(9), pp 876-892. 
%
% Notes::
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_puma560akb, mdl_stanford, mdl_twolink, mdl_coil.

function mdl_ball(N)
    
    if nargin == 0
        N = 50;
    end
    
    % create the links
    for i=1:N
        links(i) = Link([0 0 0.1, pi/2]);
        q(i) = fract(i);
    end
    
    % and build a serial link manipulator
    r = SerialLink(links, 'name', 'ball');
    
    % place the variables into the global workspace
    if nargin == 0
        assignin('base', 'coil', r);
        assignin('base', 'q', q);     
    end
    
    
end
    
    function f = fract(i)
        theta1 = 1;
        theta2 = -2/3;
        
        switch mod(i,3)
            case 1
                f = theta1;
            case 2
                f = theta2;
            case 0
                f = fract(i/3);
        end
    end