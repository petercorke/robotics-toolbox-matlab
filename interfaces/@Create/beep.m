function [] = beep(robot, note, duration);
%[] = BeepRoomba(serPort)
% Plays a song made by RoombaInit command.

% By; Joel Esposito, US Naval Academy, 2011
% sing it


    robot.flush();

    if nargin < 2
        % play song 1, set by robot constructor
        robot.write([141 1])
    else
        robot.write([140 2 1 note duration*64]);
        robot.write([141 2])
    end

