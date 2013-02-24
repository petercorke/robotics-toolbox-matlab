%MDL_HYPER2D Create model of a hyper redundant planar manipulator
%
% MDL_HYPER2D  creates the workspace variable h2d which describes the 
% kinematic characteristics of a serial link manipulator which at zero angles
% is a straight line in the XY plane.  By default has 10 joints.
%
% MDL_HYPER2D(N) as above but creates a manipulator with N joints.
%
% Also define the workspace vectors:
%   qz  joint angle vector for zero angle configuration
%
% R = MDL_HYPER2D(N) functional form of the above, returns the SerialLink object.
%
% [R,QZ] = MDL_HYPER2D(N) as above but also returns a vector of zero joint angles.
%
%
% Notes::
% - The manipulator in default pose is a straight line 1m long.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_hyper3d, mdl_puma560akb, mdl_stanford, mdl_twolink, mdl_coil.

function [r_,q_] = mdl_hyper2d(N)
    
    if nargin == 0
        N = 10;
    end
    
    % create the links
    for i=1:N
        links(i) = Link([0 0 1/N, 0]);
        q(i) = 0;
    end
    
    % and build a serial link manipulator
    r = SerialLink(links, 'name', 'hyper2d');
    
    % place the variables into the global workspace
    if nargout == 0
        assignin('base', 'h2d', r);
        assignin('base', 'qz', q);
    elseif nargout == 1
        r_ = r;
    elseif nargout == 2
        r_ = r;
        q_ = q;
    end
    
    
end