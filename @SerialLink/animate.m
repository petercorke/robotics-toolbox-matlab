%SerialLink.animate   Update a robot animation
%
% R.animate(q) updates an existing animation for the robot R.  This will have
% been created using R.plot(). Updates graphical instances of this robot in all figures.
%
% Notes::
% - Called by plot() and plot3d() to actually move the arm models.
% - Used for Simulink robot animation.
%
% See also SerialLink.plot.

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

function animate(robot, qq)

    if nargin < 3
        handles = findobj('Tag', robot.name);
    end
    
    links = robot.links;
    N = robot.n;
    
    % get handle of any existing graphical robots of same name
    %  one may have just been created above
    handles = findobj('Tag', robot.name);
    
    % MAIN DISPLAY/ANIMATION LOOP
    while true
        % animate over all instances of this robot in different axes
        
        for q=qq'  % for all configurations in trajectory
            q = q';
            for handle=handles'
                h = get(handle, 'UserData');
                
                % now draw it for a pose q
                if robot.mdh
                    % modified DH case
                    T = robot.base;
                    vert = transl(T)';
                    
                    for L=1:N
                        if robot.links(L).isprismatic()
                            % scale the box representing the prismatic joint
                            % it is based at the origin and extends in z-direction                            
                            if q(L) > 0
                                set(h.pjoint(L), 'Matrix', diag([1 1 q(L) 1]));
                            else
                                % if length is zero the matrix is singular and MATLAB complains
                                %error('Prismatic length must be > 0');
                            end
                        end
                        T = T * links(L).A(q(L));
                        set(h.link(L), 'Matrix', T); 
                        vert = [vert; transl(T)'];
                    end
                    % update the transform for link N+1 (the tool)
                    T = T * robot.tool;
                    if length(h.link) > N
                        set(h.link(N+1), 'Matrix', T);
                    end
                    vert = [vert; transl(T)'];
                else
                    % standard DH case
                    T = robot.base;
                    vert = transl(T)';
                    
                    for L=1:N
                        % for all N links
                        
                        if robot.links(L).isprismatic()
                            % scale the box representing the prismatic joint
                            % it is based at the origin and extends in z-direction
                            if q(L) > 0
                                set(h.pjoint(L), 'Matrix', diag([1 1 q(L) 1]));
                            else
                                % if length is zero the matrix is singular and MATLAB complains
                                %error('Prismatic length must be > 0');
                            end
                        end
                        
                        % now set the transform for frame {L}, this controls the displayed pose of:
                        %   the pipes associated with link L, that join {L} back to {L-1}
                        %   (optional) a prismatic joint L, that joins {L} to {L+1}
                        if h.link(L) ~= 0
                            % for plot3d, skip any 0 in the handle list
                            set(h.link(L), 'Matrix', T);
                        end
                        
                        T = T * links(L).A(q(L));
                        vert = [vert; transl(T)'];
                    end
                    % update the transform for link N+1 (the tool)
                    T = T*robot.tool;
                    if length(h.link) > N
                        set(h.link(N+1), 'Matrix', T);
                    end

                    vert = [vert; transl(T)'];
                end
                
                % now draw the shadow
                if isfield(h, 'shadow')
                    set(h.shadow, 'Xdata', vert(:,1), 'Ydata', vert(:,2), ...
                        'Zdata', h.floorlevel*ones(size(vert(:,1))));
                end
                
                % update the tool tip trail
                if isfield(h, 'trail')
                    T = robot.fkine(q);
                    robot.trail = [robot.trail; transl(T)'];
                    set(h.trail, 'Xdata', robot.trail(:,1), 'Ydata', robot.trail(:,2), 'Zdata', robot.trail(:,3));
                end
                
                % animate the wrist frame
                if ~isempty(h.wrist)
                    trplot(h.wrist, T);
                end
                
                % add a frame to the movie
                if ~isempty(h.robot.framenum)
                    % write the frame to the movie folder
                    print( '-dpng', fullfile(h.robot.moviepath, sprintf('%04d.png', h.robot.framenum)) );
                    h.robot.framenum = h.robot.framenum+1;
                end
                
                if h.robot.delay > 0
                    pause(h.robot.delay);
                    drawnow
                end
                h.q = q;
                set(handle, 'UserData', h);
            end
        end
        
        if ~h.robot.loop
            break;
        end        
    end
    
