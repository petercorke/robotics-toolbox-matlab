function v = voltage(robot);
%Indicates the voltage of Create's battery in Volts


    % By; Joel Esposito, US Naval Academy, 2011

    %Initialize preliminary return values

    robot.flush();

    robot.write([142 22]);

    v = robot.fread(1, 'uint16')/1000;
