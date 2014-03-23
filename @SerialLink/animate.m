%SerialLink.animate   Update a robot animation
%
% R.animate(q) updates an existing animation for the robot R.  This will have
% been created using R.plot().
%
% Updates graphical instances of this robot in all figures.
%
% Notes::
% - Not a general purpose method, used for Simulink robot animation.
%
% See also SerialLink.plot.

function animate(robot, qq)

    if nargin < 3
        handles = findobj('Tag', robot.name);
    end
    
    links = robot.links;
    
    % get handle of any existing graphical robots of same name
    %  one may have just been created above
    handles = findobj('Tag', robot.name);
    
    % MAIN DISPLAY/ANIMATION LOOP
    while true
        % animate over all instances of this robot in different axes
        
        for q=qq'  % for all configurations in trajectory
            
            for handle=handles'
                h = get(handle, 'UserData');
                h.q = q';
                set(handle, 'UserData', h);
                
                % now draw it for a pose q
                if robot.mdh
                    T = robot.base;
                    vert = transl(T)';
                    
                    for j=1:robot.n
                        set(h.joint(j), 'Matrix', T);
                        
                        T = T * links(j).A(q(j));
                        vert = [vert; transl(T)'];
                    end
                else
                    T = robot.base;
                    vert = transl(T)';
                    
                    for j=1:robot.n
                        if robot.links(j).isprismatic()
                            %set(h.joint(j), 'Matrix', diag([1 1 q(j) 1])*T);
                            
                            set(h.pjoint(j), 'Matrix', T*trotz(q(j))*diag([1 1 q(j) 1]));
                        end
                        set(h.joint(j), 'Matrix', T);
                        
                        T = T * links(j).A(q(j));
                        vert = [vert; transl(T)'];
                    end
                end
                
                if ~isempty(h.shadow)
                    set(h.shadow, 'Xdata', vert(:,1), 'Ydata', vert(:,2), 'Zdata', h.floorlevel*ones(size(vert(:,1))));
                end
                
                T = T * robot.tool;
                vert = [vert; transl(T)'];
                
                if ~isempty(h.wrist)
                    trplot(h.wrist, T);
                end
                
                if ~isempty(h.robot.framenum)
                    % write the frame to the movie folder
                    print( '-dpng', fullfile(h.robot.moviepath, sprintf('%04d.png', h.robot.framenum)) );
                    h.robot.framenum = h.robot.framenum+1;
                end
                
                if h.robot.delay > 0
                    pause(h.robot.delay);
                    drawnow
                end
            end
        end
        
        if ~h.robot.loop
            break;
        end
        


        
    end