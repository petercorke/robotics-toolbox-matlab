function move(robot, speed, angvel, varargin);
%travelDist(serPort, roombaSpeed, distance)
%Moves the Create the distance entered in meters. Positive distances move the
%Create foward, negative distances move the Create backwards.
%roombaSpeed should be between 0.025 and 0.5 m/s
% By; Joel Esposito, US Naval Academy, 2011

    opt.distance = [];
    opt.angle = [];
    opt.time = [];
    opt.event = [];

    opt = tb_optparse(opt, varargin);
    
    % enforce some limits

    if (speed < 0) %Speed given by user shouldn't be negative
        disp('WARNING: Speed inputted is negative. Should be positive. Taking the absolute value');
        speed = abs(speed);
    end
    
    robot.flush();

    if ~all(isnan([opt.distance opt.angle opt.time opt.event]))
        % we have a limit defined


        if ~isempty(opt.distance)
            robot.write([152 9]);
            if (opt.distance < 0) 
                %Definition of SetFwdVelRAdius Roomba, speed has to be negative to go backwards.
                % Takes care of this case. User shouldn't worry about negative speeds
                speed = -speed;
            end
            SetFwdVelAngVelCreate(robot, speed, angvel);
            robot.write([156]);
                
            robot.fwrite(opt.distance*1000, 'int16');
        elseif ~isempty(opt.angle)
            robot.write([152 9]);
            SetFwdVelAngVelCreate(robot, speed, angvel);
            robot.write([157]);
            robot.fwrite(opt.distance, 'int16');
        elseif ~isempty(opt.time)
            robot.write([152 14]);
            SetFwdVelAngVelCreate(robot, speed, angvel);
            robot.write([155 opt.time*10]);
        elseif ~isempty(opt.event)
            robot.write([152 14]);
            SetFwdVelAngVelCreate(robot, speed, angvel);
            robot.write([158 opt.event]);
        end

        
        % tell it to stop moving
        SetFwdVelAngVelCreate(robot, 0, 0);
        
        robot.write([142 1]); % return a sensor value

        pause(robot.delay)
        
        robot.write([153]); % play the script
        
        % wait for motion to finish
        disp('waiting');
        while( robot.serPort.BytesAvailable() ==0)
            %disp('waiting to finish')
            pause(robot.delay)
        end
        pause(robot.delay)
    
    else
        % no limit, constant velocity forever
        % set robot moving
        SetFwdVelAngVelCreate(robot, speed, angvel);
    end
    pause(robot.delay)


    
end

function [] = SetFwdVelAngVelCreate(robot, FwdVel, AngVel )
%[] = SetFwdVelAngVelCreate(serPort, FwdVel, AngVel )
%  Specify forward velocity in meters/ sec
%  [-0.5, 0.5].   Specify Angular Velocity in rad/sec.  Negative velocity is backward/Clockwise.  Caps overflow.
%  Note that the wheel speeds are capped at .5 meters per second.  So it is possible to 
% specify speeds that cannot be acheived.  Warning is displayed. 
% Only works with Create I think...not Roomba
% By; Joel Esposito, US Naval Academy, 2011
    
    robot.flush();

    d = 0.258; % wheel baseline
    wheelVel = inv([.5 .5; 1/d -1/d])*[FwdVel; AngVel];
    rightWheelVel = min( max(1000* wheelVel(1), -500) , 500);
    leftWheelVel = min( max(1000* wheelVel(2), -500) , 500);
    if ( abs(rightWheelVel) ==500) |  ( abs(leftWheelVel) ==500)
       disp('Warning: desired velocity combination exceeds limits') 
    end

    %[leftWheelVel rightWheelVel]

    robot.write([145]);
    robot.fwrite(rightWheelVel, 'int16'); 
    robot.fwrite(leftWheelVel, 'int16');
    pause(robot.delay)
end
