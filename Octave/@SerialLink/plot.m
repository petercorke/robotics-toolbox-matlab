%PLOT	Graphical robot animation
%
%	PLOT(ROBOT, Q)
%	PLOT(ROBOT, Q, options)
%
%	Displays a graphical animation of a robot based on the
%	kinematic model.  A stick figure polyline joins the origins of
%	the link coordinate frames.
%	The robot is displayed at the joint angle Q, or if a matrix it is
%	animated as the robot moves along the trajectory.
%
%	The graphical robot object holds a copy of the robot object and
%	the graphical element is tagged with the robot's name (.name method).
%	This state also holds the last joint configuration 
%	drawn (.q method).
%
%	If no robot of this name is currently displayed then a robot will
%	be drawn in the current figure.  If the robot already exists then
%	that graphical model will be found and moved.
%
%	MULTIPLE VIEWS OF THE SAME ROBOT
%	If one or more plots of this robot already exist then these will all
%	be moved according to the argument Q.
%
%	MULTIPLE ROBOTS
%	Multiple robots can be displayed in the same plot, by using "hold on"
%	before calls to plot(robot).
%
%	options is a list of any of the following:
%	'workspace' [xmin, xmax ymin ymax zmin zmax]
%	'perspective' 'ortho'	controls camera view mode
%	'erase' 'noerase'	controls erasure of arm during animation
%	'base' 'nobase'		controls display of base 'pedestal'
%	'loop' 'noloop'		controls display of base 'pedestal'
%	'wrist' 'nowrist'	controls display of wrist
%	'name', 'noname'	display the robot's name near the first joint
%	'shadow' 'noshadow'	controls display of shadow
%	'xyz' 'noa'		wrist axis label
%	'joints' 'nojoints'	controls display of joints
%	'mag' scale		annotation scale factor
%
%	The options come from 3 sources and are processed in the order:
%	1. Cell array of options returned by the function PLOTBOTOPT
%	2. Cell array of options returned by the .plotopt method of the
%	   robot object.  These are set by the .plotopt method.
%	3. List of arguments in the command line.
%
% GRAPHICAL ANNOTATIONS:
%
%	The basic stick figure robot can be annotated with
%		shadow on the floor
%		XYZ wrist axes and labels
%		joint cylinders and axes
%
%	All of these require some kind of dimension and this is determined
%	using a simple heuristic from the workspace dimensions.  This dimension
%	can be changed by setting the multiplicative scale factor using the
%	'mag' option above.
%
% GETTING GRAPHICAL ROBOT STATE:
%	q = PLOT(ROBOT)
%	Returns the joint configuration from the state of an existing 
%	graphical representation of the specified robot.  If multiple
%	instances exist, that of the first one returned by findobj() is
%	given.

%
%	See also PLOTBOTOPT, DRIVEBOT, FKINE, ROBOT.

%	Copright (C) Peter Corke 1993

% HANDLES:
%
%  A robot comprises a bunch of individual graphical elements and these are 
% kept in a structure which can be stored within the .handle element of a
% robot object.
%	h.robot		the robot stick figure
%	h.shadow	the robot's shadow
%	h.x		wrist vectors
%	h.y
%	h.z
%	h.xt		wrist vector labels
%	h.yt
%	h.zt
%
%  The plot function returns a new robot object with the handle element set.
%
% For the h.robot object we additionally:
%	save this new robot object as its UserData
%	tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
% and update them.

% MOD.HISTORY
%	12/94	make axis scaling adjust to robot kinematic params
%	4/99	use objects
%	2/01	major rewrite, axis names, drivebot, state etc.


% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%
% Copyright (C) 1993-2011, by Peter I. Corke
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
function rnew = plot(robot, tg, varargin)

	%
	% q = PLOT(robot)
	% return joint coordinates from a graphical robot of given name
	%
	if nargin == 1,
		rh = findobj('Tag', robot.name);
		if ~isempty(rh),
			r = get(rh(1), 'UserData');
			rnew = r.q;
		end
		return
	end

	np = numrows(tg);
	n = robot.n;

	if numcols(tg) ~= n,
		error('Insufficient columns in q')
	end

	if ~isfield(robot, 'handles'),
		%
		% if there are no handles in this object we assume it has
		% been invoked from the command line not drivebot() so we
		% process the options
		%

		%%%%%%%%%%%%%% process options
		erasemode = 'xor';
		joints = 1;
		wrist = 1;
		repeat = 1;
		shadow = 1;
		wrist = 1;
		dims = [];
		base = 0;
		wristlabel = 'xyz';
		projection = 'orthographic';
		magscale = 1;
		name = 1;

		% read options string in the order
		%	1. robot.plotopt
		%	2. the M-file plotbotopt if it exists
		%	3. command line arguments
		if exist('plotbotopt', 'file') == 2,
			options = plotbotopt;
			options = [options robot.plotopt varargin];
		else
			options = [robot.plotopt varargin];
		end
		i = 1;
		while i <= length(options),
			switch lower(options{i}),
			case 'workspace'
				dims = options{i+1};
				i = i+1;
			case 'mag'
				magscale = options{i+1};
				i = i+1;
			case 'perspective'
				projection = 'perspective';
			case 'ortho'
				projection = 'orthographic';
			case 'erase'
				erasemode = 'xor';
			case 'noerase'
				erasemode = 'none';
			case 'base'
				base = 1;
			case 'nobase'
				base = 0;
			case 'loop'
				repeat = Inf;
			case 'noloop'
				repeat = 1;
			case 'name',
				name = 1;
			case 'noname',
				name = 0;
			case 'wrist'
				wrist = 1;
			case 'nowrist'
				wrist = 0;
			case 'shadow'
				shadow = 1;
			case 'noshadow'
				shadow = 0;
			case 'xyz'
				wristlabel = 'XYZ';
			case 'noa'
				wristlabel = 'NOA';
			case 'joints'
				joints = 1;
			case 'nojoints'
				joints = 0;
			otherwise
				error(['unknown option: ' options{i}]);
			end
			i = i+1;
		end

		if isempty(dims),
			%
			% simple heuristic to figure the maximum reach of the robot
			%
			L = robot.links;
			reach = 0;
			for i=1:n,
				reach = reach + abs(L(i).a) + abs(L(i).d);
			end
			dims = [-reach reach -reach reach -reach reach];
			mag = reach/10;
		else
			reach = min(abs(dims));
		end
		mag = magscale * reach/10;
	end

	%
	% setup an axis in which to animate the robot
	%
	if isempty(robot.handle) & isempty(findobj(gcf, 'Tag', robot.name)),
		if ~ishold,
			clf
			axis(dims);
		end
		figure(gcf);		% bring to the top
		xlabel('X')
		ylabel('Y')
		zlabel('Z')
		set(gca, 'drawmode', 'fast');
		grid on

		zlim = get(gca, 'ZLim');
		h.zmin = zlim(1);

		if base,
			b = transl(robot.base);
			line('xdata', [b(1);b(1)], ...
				'ydata', [b(2);b(2)], ...
				'zdata', [h.zmin;b(3)], ...
				'LineWidth', 4, ...
				'color', 'red');
		end
		if name,
			b = transl(robot.base);
			text(b(1), b(2)-mag, [' ' robot.name])
		end
		% create a line which we will
		% subsequently modify.  Set erase mode to xor for fast
		% update
		%
		h.robot = line(robot.lineopt{:}, ...
			'Erasemode', erasemode);
		if shadow == 1,
			h.shadow = line(robot.shadowopt{:}, ...
				'Erasemode', erasemode);
		end

		if wrist == 1,	
			h.x = line('xdata', [0;0], ...
				'ydata', [0;0], ...
				'zdata', [0;0], ...
				'color', 'red', ...
				'erasemode', 'xor');
			h.y = line('xdata', [0;0], ...
				'ydata', [0;0], ...
				'zdata', [0;0], ...
				'color', 'green', ...
				'erasemode', 'xor');
			h.z = line('xdata', [0;0], ...
				'ydata', [0;0], ...
				'zdata', [0;0], ...
				'color', 'blue', ...
				'erasemode', 'xor');
			h.xt = text(0, 0, wristlabel(1), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
			h.yt = text(0, 0, wristlabel(2), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
			h.zt = text(0, 0, wristlabel(3), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
		end

		%
		% display cylinders (revolute) or boxes (pristmatic) at
		% each joint, as well as axis centerline.
		%
		if joints == 1,
			L = robot.links;
			for i=1:robot.n,

				% cylinder or box to represent the joint
				if L(i).sigma == 0,
					N = 8;
				else
					N = 4;
				end
				
				[xc,yc,zc] = cylinder([mag/4, mag/4], N);
				zc(zc==0) = -mag/2;
				zc(zc==1) = mag/2;

				% add the surface object and color it
				h.joint(i) = surface(xc,yc,zc);
				%set(h.joint(i), 'erasemode', 'xor');
				set(h.joint(i), 'FaceColor', 'blue');

				% build a matrix of coordinates so we
				% can transform the cylinder in animate()
				% and hang it off the cylinder
				xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,2*N+2)]; 
				set(h.joint(i), 'UserData', xyz);

				% add a dashed line along the axis
				h.jointaxis(i) = line('xdata', [0;0], ...
					'ydata', [0;0], ...
					'zdata', [0;0], ...
					'color', 'blue', ...
					'linestyle', '--', ...
					'erasemode', 'xor');
				
			end
		end
		h.mag = mag;
		robot.handle = h;
		set(h.robot, 'Tag', robot.name);
		set(h.robot, 'UserData', robot);
	end


	% save the handles in the passed robot object

	rh = findobj('Tag', robot.name);
	for r=1:repeat,
	    for p=1:np,
		for r=rh',
			animate( get(r, 'UserData'), tg(p,:));
		end
	    end
	end

	% save the last joint angles away in the graphical robot
	for r=rh',
		rr = get(r, 'UserData');
		rr.q = tg(end,:);
		set(r, 'UserData', rr);
	end

	if nargout > 0,
		rnew = robot;
	end

function animate(robot, q)

	n = robot.n;
	h = robot.handle;
	L = robot.links;

	mag = h.mag;

	b = transl(robot.base);
	x = b(1);
	y = b(2);
	z = b(3);

	xs = b(1);
	ys = b(2);
	zs = h.zmin;

	% compute the link transforms, and record the origin of each frame
	% for the animation.
	t = robot.base;
	Tn = t;
	for j=1:n,
		Tn(:,:,j) = t;
		

		t = t * L(j).A(q(j));

		x = [x; t(1,4)];
		y = [y; t(2,4)];
		z = [z; t(3,4)];
		xs = [xs; t(1,4)];
		ys = [ys; t(2,4)];
		zs = [zs; h.zmin];
	end
	t = t *robot.tool;

	%
	% draw the robot stick figure and the shadow
	%
	set(h.robot,'xdata', x, 'ydata', y, 'zdata', z);
	if isfield(h, 'shadow'),
		set(h.shadow,'xdata', xs, 'ydata', ys, 'zdata', zs);
	end
	

	%
	% display the joints as cylinders with rotation axes
	%
	if isfield(h, 'joint'),
		xyz_line = [0 0; 0 0; -2*mag 2*mag; 1 1];

		for j=1:n,
			% get coordinate data from the cylinder
			xyz = get(h.joint(j), 'UserData');
			xyz = Tn(:,:,j) * xyz;
			ncols = numcols(xyz)/2;
			xc = reshape(xyz(1,:), 2, ncols);
			yc = reshape(xyz(2,:), 2, ncols);
			zc = reshape(xyz(3,:), 2, ncols);

			set(h.joint(j), 'Xdata', xc, 'Ydata', yc, ...
				'Zdata', zc);

			xyzl = Tn(:,:,j) * xyz_line;
			set(h.jointaxis(j), 'Xdata', xyzl(1,:), 'Ydata', xyzl(2,:), 'Zdata', xyzl(3,:));
			xyzl = Tn(:,:,j) * xyz_line;
			h.jointlabel(j) = text(xyzl(1,1),  xyzl(2,1) , xyzl(3,1), num2str(j), 'HorizontalAlignment', 'Center');
		end
	end

	%
	% display the wrist axes and labels
	%
	if isfield(h, 'x'),
		%
		% compute the wrist axes, based on final link transformation
		% plus the tool transformation.
		%
		xv = t*[mag;0;0;1];
		yv = t*[0;mag;0;1];
		zv = t*[0;0;mag;1];

		%
		% update the line segments, wrist axis and links
		%
		set(h.x,'xdata',[t(1,4) xv(1)], 'ydata', [t(2,4) xv(2)], ...
			'zdata', [t(3,4) xv(3)]);
		set(h.y,'xdata',[t(1,4) yv(1)], 'ydata', [t(2,4) yv(2)], ...
			 'zdata', [t(3,4) yv(3)]);
		set(h.z,'xdata',[t(1,4) zv(1)], 'ydata', [t(2,4) zv(2)], ...
			 'zdata', [t(3,4) zv(3)]);
		
		 wristlabel = "XYZ";
		% text(xv(1),xv(2),xv(3), ['X'])
		% text(yv(1),yv(2),yv(3), ['Y'])
		% text(zv(1),zv(2),zv(3), ['Z'])
		h.xt = text(xv(1),xv(2),xv(3), wristlabel(1), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
		h.yt = text(yv(1),yv(2),yv(3), wristlabel(2), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
		h.zt = text(zv(1),zv(2),zv(3), wristlabel(3), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
		
		% set(h.zt, 'rotation', zv(1:3));
		% set(h.xt, 'Position', xv(1:3));
		% set(h.yt, 'Position', yv(1:3));
	end
	
	drawnow
