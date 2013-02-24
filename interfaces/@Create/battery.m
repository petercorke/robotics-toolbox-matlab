function Percent = battery(robot)
% Displays the current percent charge remaining in Create's Battery 

% By; Joel Esposito, US Naval Academy, 2011

    robot.flush();

    robot.write([142 25]);
    charge =  robot.fread(1, 'uint16');
    robot.write([142 26]);
    capacity =  robot.fread(1, 'uint16');

    percent=charge/capacity*100;

    if nargout == 0
        fprintf('battery  %.0f%%\n', percent);
    else
        Percent = percent;
    end
