function mode(robot, mode)

    if strcmp(mode, 'off')
        robot.write([128]);
        robot.cur_mode = 'passive';
    elseif strcmp(mode, 'passive');
        robot.write([128]);
        robot.cur_mode = 'passive';
    elseif strcmp(mode, 'safe');
        robot.write([131]);
        robot.cur_mode = 'safe';
    elseif strcmp(mode, 'full');
        robot.write([132]);
        robot.cur_mode = 'full';
    else
        error('no such mode %s', mode);
    end
    pause(0.1);
