% init the connection
disp('Connecting ... ')
% brick usb init
b = Brick('ioType','usb');
% beep to indicate connection
b.beep();
% print information
disp('===================================')
disp('EV3 example code')
disp('===================================')
disp('u - increase motor speed')
disp('d - decrease motor speed')
disp('t - start the motor')
disp('s - stop the motor')
disp('b - beep')
disp('o - output the tachometer')
disp('c - clear the tachometer')
disp('v - output the brick voltage')
disp('f - quit the program');
disp('===================================')
% user intput
userIn = '';
% motor power
motorPower = 10;
% set motor power and start
b.outputPower(0,Device.MotorA,motorPower)
b.outputStart(0,Device.MotorA)
while(~strcmp(userIn,'f'))
    % get input
    userIn = input('> Input(u,d,t,s,b,o,c,v,f): ','s');
    % increase speed
    if (userIn == 'u')
        motorPower = motorPower+10;
        if motorPower >= 100
            motorPower = 100;
        end
        b.outputPower(0,Device.MotorA,motorPower)
        disp(['> Motor Power: ' num2str(motorPower)]);
    end
     % decrease speed
    if (userIn == 'd')
        motorPower = motorPower-10;
        if motorPower <= -100
            motorPower = -100;
        end
        b.outputPower(0,Device.MotorA,motorPower)
        disp(['> Motor Power: ' num2str(motorPower)]);
    end
    % start the motor
    if (userIn == 't')
        b.outputStart(0,Device.MotorA)
        disp('> Motor Started');
    end
    % stop the motor
    if (userIn == 's')
        b.outputStop(0,Device.MotorA,0)
        disp('> Motor Stopped');
    end
    % beep test
    if (userIn == 'b')
        b.beep();
        disp('> Beep');
    end
    % output the tachometer
    if (userIn == 'o')
        tacho = b.outputGetCount(0,Device.MotorA);
        disp(['> Tachometer: ' num2str(tacho)]);
    end
    % clear the tachometer
    if (userIn == 'c')
        b.outputClrCount(0,Device.MotorA)
        disp('> Cleared tacho');
    end
    % voltage output
    if (userIn == 'v')
        v = b.uiReadVbatt;
        disp(['> Brick voltage: ' num2str(v) 'V']);
    end
end
% delete the brick object
delete(b)