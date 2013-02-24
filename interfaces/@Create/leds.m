function [] = leds(robot, LED, Color, Intensity)
%[] = SetLEDsRoomba(serPort, LED,Color, Intensity)
% Manipulates LEDS
% LED = 0 for Play or 1 for Advance 
% Color determines which color(Red/Green) that the Power LED will
% illuminate as, from 0-100%
% 0 is pure green, 100 is pure red.
% Intensity determines how bright the Power LED appears from 1-100%


% By; Joel Esposito, US Naval Academy, 2011

    switch LED
    case 0
        LED=bin2dec('00000010');
    case 1
        LED=bin2dec('00001000');
    otherwise
        error('LED is 0 or 1');
    end
       
    aColor= (Color/100)*255;
    aIntensity = (Intensity/100)*255;

    robot.write([139 LED aColor aIntensity]);

    pause(0.05);

