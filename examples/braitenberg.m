function p = braitenberg(world, robot)
    % these are coordinates (x,y)
  
    % display the world, obstacle, robot and target
    image(world/max(world(:))*100)
    colormap(gray);
    set(gca, 'Ydir', 'normal');
    xlabel('x');
    ylabel('y');
    hold on

    if nargin < 2
        disp('Click on a start point');
        [x,y] = ginput(1);
        robot = [x y];
    end

    if length(robot) == 2
        T = transl( [robot 0] );
    else
        T = transl( [robot(1:2) 0] ) * trotz(robot(3));
    end


    p = [];

    sensorOffset = 2;
    forwardStep = 1.0;

    while 1

        % read the left and right sensor
        leftSensor = sensorRead(world, T, transl(0, -sensorOffset, 0));
        rightSensor = sensorRead(world, T, transl(0, sensorOffset, 0));

        % change our heading based on the ratio of the sensors
        dth = rightSensor / leftSensor - 1;

        % bad sensor reading means we fell off the world
        if isnan(dth)
            break
        end

        % update the robot's pose
        dT = trotz(dth) * transl(forwardStep, 0, 0);
        T = T * dT;

        % display the robot's position
        xyz = transl( T );      % get position
        rpy = tr2rpy( T );      % get heading angle
        plot(xyz(1), xyz(2), 'g.');

        p = [p; xyz(1:2)' rpy(3)];
        pause(0.1);

        if numrows(p) > 150
            break;
        end
    end
    hold off

end

% return the sensor reading for the given robot pose and relative pose
% of the sensor.
function s = sensorRead(field, robot, Tsens)
    xyz = transl( robot * Tsens );

    s = interp2(field, xyz(1), xyz(2));
end
