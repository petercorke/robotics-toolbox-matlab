%SerialLink.plot3d Graphical display and animation of solid model robot
%
% R.plot3d(Q, options) displays and animates a solid model of the robot.
% The robot is displayed at the joint angle Q (1xN), or
% if a matrix (MxN) it is animated as the robot moves along the M-point trajectory.
%
% Options::
%
% 'color',C         A cell array of color names, one per link.  These are
%                   mapped to RGB using colorname().  If not given, colors
%                   come from the axis ColorOrder property.
% 'alpha',A         Set alpha for all links, 0 is transparant, 1 is opaque
%                   (default 1)
% 'path',P          Overide path to folder containing STL model files
% 'workspace', W    Size of robot 3D workspace, W = [xmn, xmx ymn ymx zmn zmx]
% 'floorlevel',L    Z-coordinate of floor (default -1)
%-
% 'delay',D         Delay betwen frames for animation (s)
% 'fps',fps         Number of frames per second for display, inverse of 'delay' option
% '[no]loop'        Loop over the trajectory forever
% '[no]raise'       Autoraise the figure
% 'movie',M         Save frames as files in the folder M
%-
% 'scale',S         Annotation scale factor
% 'ortho'           Orthographic view (default)
% 'perspective'     Perspective view
% 'view',V          Specify view V='x', 'y', 'top' or [az el] for side elevations,
%                   plan view, or general view by azimuth and elevation
%                   angle. 
%-
% '[no]wrist'       Enable display of wrist coordinate frame
% 'xyz'             Wrist axis label is XYZ
% 'noa'             Wrist axis label is NOA
% '[no]arrow'       Display wrist frame with 3D arrows
%-
% '[no]tiles'       Enable tiled floor (default true)
% 'tilesize',S      Side length of square tiles on the floor (default 0.2)
% 'tile1color',C   Color of even tiles [r g b] (default [0.5 1 0.5]  light green)
% 'tile2color',C   Color of odd tiles [r g b] (default [1 1 1] white)
%-
% '[no]jaxes'       Enable display of joint axes (default true)
% '[no]joints'      Enable display of joints
%-
% '[no]base'        Enable display of base shape
%
% Notes::
% - Solid models of the robot links are required as STL ascii format files,
%   with extensions .stl
% - Suitable STL files can be found in the package ARTE: A ROBOTICS TOOLBOX
%   FOR EDUCATION by Arturo Gil, https://arvc.umh.es/arte
% - The root of the solid models is an installation of ARTE with an empty
%   file called arte.m at the top level
% - Each STL model is called 'linkN'.stl where N is the link number 0 to N
% - The specific folder to use comes from the SerialLink.model3d property
% - The path of the folder containing the STL files can be specified using
%   the 'path' option
% - The height of the floor is set in decreasing priority order by:
%   - 'workspace' option, the fifth element of the passed vector
%   - 'floorlevel' option
%   - the lowest z-coordinate in the link1.stl object
%
% Authors::
% - Peter Corke, based on existing code for plot()
% - Bryan Moutrie, demo code on the Google Group for connecting ARTE and RTB
% - Don Riley, function rndread() extracted from cad2matdemo (MATLAB
%   File Exchange)
%
% See also SerialLink.plot, plotbotopt3d, SerialLink.animate, SerialLink.teach, SerialLink.fkine.

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


function plot3d(robot, q, varargin)
    
    
    if robot.mdh
        error('RTB:plot3d:badmodel', '3D models are defined for standard, not modified, DH parameters');
    end
    
    clf
    opt = plot_options(robot, varargin);
    
    %-- load the shape if need be
    
    nshapes = robot.n+1;
    
    if isempty(robot.faces)
        % no 3d model defined, let's try to load one
        
        if isempty(opt.path)
            % first find the path to the models
            pth = which('arte.m');
            if ~pth
                error('RTB:plot3d:nomodel', 'no 3D model found, install the RTB contrib zip file');
            end
            
            % find the path to this specific model
            pth = fullfile(fileparts(pth), 'robots', robot.model3d);
        else
            pth = opt.path;
        end
        
        % now load the STL files
        robot.points = cell(1, robot.n+1);
        robot.faces = cell(1, robot.n+1);
        for i=1:nshapes
            [F,P] = rndread( fullfile(pth, sprintf('link%d.stl', i-1)) );
            robot.points{i} = P;
            robot.faces{i} = F;
        end
    end
    
    % if a base is specified set the floor height to this
    if isempty(opt.workspace)
        % workspace not provided, fall through the options for setting floor level
        if ~isempty(opt.floorlevel)
            opt.ws(5) = opt.floorlevel;
        elseif opt.base
            mn = min( robot.points{1} );
            opt.ws(5) = mn(3);
        end
    end
    opt.floorlevel = opt.ws(5);

% TODO
% should test if the plot exists, pinch the logic from plot()

    %-- set up to plot
    % create an axis
    ish = ishold();
    if ~ishold
        % if hold is off, set the axis dimensions
        axis(opt.ws);
        set(gca, 'ZLimMode', 'manual');
        axis(opt.ws);
        hold on
    end
    

    if opt.raise
        % note this is a very time consuming operation
        figure(gcf);
    end
    
    if strcmp(opt.projection, 'perspective')
        set(gca, 'Projection', 'perspective');
    end
    
    grid on
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    %--- create floor if required
    if opt.tiles
        create_tiled_floor(opt);
    end
    
    %--- configure view and lighting
    if isstr(opt.view)
        switch opt.view
            case 'top'
                view(0, 90);
            case 'x'
                view(0, 0);
            case 'y'
                view(90, 0)
            otherwise
                error('rtb:plot3d:badarg', 'view must be: x, y, top')
        end
    elseif isnumeric(opt.view) && length(opt.view) == 2
        view(opt.view)
    else
        campos([2 2 1]);
    end

    daspect([1 1 1]);
    light('Position', [0 0 opt.reach*2]);
    light('Position', [1 0.5 1]);
    
    
    %-- figure the colors for each shape 
    if isempty(opt.color)
        % if not given, use the axis color order
        C = get(gca,'ColorOrder');
    else
        C = [];
        for c=opt.color
            C = [C; colorname(c{1})];
        end
    end
    

    %--- create the robot
    %  one patch per shape, use hgtransform to animate them later
    group = hggroup('Tag', robot.name);
    ncolors = numrows(C);
    
    h = [];
    for link=0:robot.n    
        if link == 0
            if ~opt.base
                continue;
            end
            
            patch('Faces', robot.faces{link+1}, 'Vertices', robot.points{link+1}, ...
                'FaceColor', C(mod(link,ncolors)+1,:), 'EdgeAlpha', 0, 'FaceAlpha', opt.alpha);
        else
            h.link(link) = hgtransform('Tag', sprintf('link%d', link), 'Parent', group);

            patch('Faces', robot.faces{link+1}, 'Vertices', robot.points{link+1}, ...
                'FaceColor', C(mod(link,ncolors)+1,:), 'EdgeAlpha', 0, 'FaceAlpha', opt.alpha, ...
            'Parent', h.link(link));
        end
    end
    
    % enable mouse-based 3D rotation
    rotate3d on
    
    h.wrist = [];  % HACK, should be trplot
    h.robot = robot;
    h.link = [0 h.link];
    set(group, 'UserData', h);
    
    robot.animate(q);
    
    if ~ish
        hold off
    end
end

function opt = plot_options(robot, optin)
    opt.color = [];
    opt.path = [];  % override path
    opt.alpha = 1;
    
        % timing/looping
    opt.delay = 0.1;
    opt.fps = [];
    opt.loop = false;
    
    opt.raise = false;
    
    % general appearance
    opt.scale = 1;
    
    opt.workspace = [];
    opt.floorlevel = [];

    opt.name = true;
    opt.projection = {'ortho', 'perspective'};
    opt.view = [];

    
    % tiled floor
    opt.tiles = true;
    opt.tile1color = [0.5 1 0.5];  % light green
    opt.tile2color = [1 1 1];  % white
    opt.tilesize = 0.2;
    
     
    % the base or pedestal
    opt.base = true;

    % wrist
    opt.wrist = true;
    opt.wristlabel = {'xyz', 'noa'};
    opt.arrow = true;
    
    % joint rotation axes
    opt.jaxes = false;
    
    
    % misc
    opt.movie = [];

    % build a list of options from all sources
    %   1. the M-file plotbotopt if it exists
    %   2. robot.plotopt
    %   3. command line arguments
    if exist('plotbotopt3d', 'file') == 2
        options = [plotbotopt3d robot.plotopt3d optin];
    else
        options = [robot.plotopt3d optin];
    end
    
    % parse the options
    [opt,args] = tb_optparse(opt, options);
    if ~isempty(args)
        error(['unknown option: ' args{1}]);
    end
    
    if ~isempty(opt.fps)
        opt.delay = 1/opt.fps;
    end
    % figure the size of the figure
    
    if isempty(opt.workspace)
        %
        % simple heuristic to figure the maximum reach of the robot
        %
        L = robot.links;
        if any(L.isprismatic)
            error('Prismatic joint(s) present: requires the ''workspace'' option');
        end
        reach = 0;
        for i=1:robot.n
            reach = reach + abs(L(i).a) + abs(L(i).d);
        end
        
        % if we have a floor, quantize the reach to a tile size
        if opt.tiles
            reach = opt.tilesize * ceil(reach/opt.tilesize);
        end
        
        % now create a 3D volume based on this reach
        opt.ws = [-reach reach -reach reach -reach reach];
        
        % if a floorlevel has been given, ammend the 3D volume
        if ~isempty(opt.floorlevel)
            opt.ws(5) = opt.floorlevel;
        end
    else
        % workspace is provided
        reach = min(abs(opt.workspace));
        if opt.tiles
            % set xy limits to be integer multiple of tilesize
            opt.ws(1:4) = opt.tilesize * round(opt.workspace(1:4)/opt.tilesize);
            opt.ws(5:6) = opt.workspace(5:6);
        end
    end
    
    opt.reach = reach;
    
    % update the fundamental scale factor (given by the user as a multiplier) by a length derived from
    % the overall workspace dimension
    %  we need that a lot when creating the robot model
    opt.scale = opt.scale * reach/40;
    
    % deal with a few options that need to be stashed in the SerialLink object
    % movie mode has not already been flagged
    if opt.movie
        robot.framenum = 0;
        robot.moviepath = opt.movie;
    else
        robot.framenum = [];
    end 
    
    if ~isempty(opt.movie)
        mkdir(opt.movie);
        framenum = 1;
    end
    
    robot.delay = opt.delay;
    robot.loop = opt.loop;   
end

% Extracted from cad2matdemo
% http://www.mathworks.com/matlabcentral/fileexchange/3642-cad2matdemo-m/content/cad2matR2.zip
%
% Copyright (c) 2003, Don Riley
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
%     * Neither the name of the Walla Walla University nor the names
%       of its contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

function [fout, vout, cout] = rndread(filename)
    
    % Reads CAD STL ASCII files, which most CAD programs can export.
    % Used to create Matlab patches of CAD 3D data.
    % Returns a vertex list and face list, for Matlab patch command.
    %
    % filename = 'hook.stl';  % Example file.
    %
    fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.
    if fid == -1
        error('File could not be opened, check name or path.')
    end
    %
    % Render files take the form:
    %
    %solid BLOCK
    %  color 1.000 1.000 1.000
    %  facet
    %      normal 0.000000e+00 0.000000e+00 -1.000000e+00
    %      normal 0.000000e+00 0.000000e+00 -1.000000e+00
    %      normal 0.000000e+00 0.000000e+00 -1.000000e+00
    %    outer loop
    %      vertex 5.000000e-01 -5.000000e-01 -5.000000e-01
    %      vertex -5.000000e-01 -5.000000e-01 -5.000000e-01
    %      vertex -5.000000e-01 5.000000e-01 -5.000000e-01
    %    endloop
    % endfacet
    %
    % The first line is object name, then comes multiple facet and vertex lines.
    % A color specifier is next, followed by those faces of that color, until
    % next color line.
    %
    CAD_object_name = sscanf(fgetl(fid), '%*s %s');  %CAD object name, if needed.
    %                                                %Some STLs have it, some don't.
    vnum=0;       %Vertex number counter.
    report_num=0; %Report the status as we go.
    VColor = 0;
    %
    while feof(fid) == 0                    % test for end of file, if not then do stuff
        tline = fgetl(fid);                 % reads a line of data from file.
        fword = sscanf(tline, '%s ');       % make the line a character string
        % Check for color
        if strncmpi(fword, 'c',1) == 1;    % Checking if a "C"olor line, as "C" is 1st char.
            VColor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face.
        end                                % Keep this color, until the next color is used.
        if strncmpi(fword, 'v',1) == 1;    % Checking if a "V"ertex line, as "V" is 1st char.
            vnum = vnum + 1;                % If a V we count the # of V's
            report_num = report_num + 1;    % Report a counter, so long files show status
            if report_num > 249;
                %disp(sprintf('Reading vertix num: %d.',vnum));
                report_num = 0;
            end
            v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % & if a V, get the XYZ data of it.
            c(:,vnum) = VColor;              % A color for each vertex, which will color the faces.
        end                                 % we "*s" skip the name "color" and get the data.
    end
    %   Build face list; The vertices are in order, so just number them.
    %
    fnum = vnum/3;      %Number of faces, vnum is number of vertices.  STL is triangles.
    flist = 1:vnum;     %Face list of vertices, all in order.
    F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
    %
    %   Return the faces and vertexs.
    %
    fout = F';  %Orients the array for direct use in patch.
    vout = v';  % "
    cout = c';
    %
    fclose(fid);
end

% draw a tiled floor in the current axes
function create_tiled_floor(opt)
     
    xmin = opt.ws(1);
    xmax = opt.ws(2);
    ymin = opt.ws(3);
    ymax = opt.ws(4);
    
    % create a colored tiled floor
    xt = xmin:opt.tilesize:xmax;
    yt = ymin:opt.tilesize:ymax;
    Z = opt.floorlevel*ones( numel(yt), numel(xt));
    C = zeros(size(Z));
    [r,c] = ind2sub(size(C), 1:numel(C));
    C = bitand(r+c,1);
    C = reshape(C, size(Z));
    C = cat(3, opt.tile1color(1)*C+opt.tile2color(1)*(1-C), ...
        opt.tile1color(2)*C+opt.tile2color(2)*(1-C), ...
        opt.tile1color(3)*C+opt.tile2color(3)*(1-C));
    [X,Y] = meshgrid(xt, yt);
    surface(X, Y, Z, C, ...
        'FaceColor','texturemap',...
        'EdgeColor','none',...
        'CDataMapping','direct');
end
