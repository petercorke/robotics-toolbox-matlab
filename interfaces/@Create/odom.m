function d = odom(robot)
% Displays the current percent charge remaining in Create's Battery 

% By; Joel Esposito, US Naval Academy, 2011

    robot.flush();

    robot.write([142 19]);
    dd =  robot.fread(1, 'uint16');
    robot.write([142 20]);
    dtheta =  robot.fread(1, 'uint16');

    d = [dd/1000 dtheta];
