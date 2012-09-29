%MDL_COIL Create model of a coil manipulator
%
% MDL_COIL  creates the workspace variable coil which describes the 
% kinematic characteristics of a serial link manipulator that folds into
% a helix shape.  By default has 50 joints.
%
% MDL_BALL(N) as above but creates a manipulator with N joints.
%
% Also define the workspace vectors:
%   q  joint angle vector for default helical configuration
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
% See also SerialLink, mdl_puma560akb, mdl_stanford, mdl_twolink, mdl_ball.
function mdl_coil(N)
    
    if nargin == 0
        N = 50;
    end
    
    % create the links
    for i=1:N
        links(i) = Link('d', 0, 'a', 1/N, 'alpha', 5*pi/N);
    end
    
    % and build a serial link manipulator
    r = SerialLink(links, 'name', 'coil');
    
    % place the variables into the global workspace
    if nargin == 0
        assignin('base', 'coil', r);
        assignin('base', 'q', 10*pi/N*ones(1,N));
    end