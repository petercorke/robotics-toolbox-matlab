function turnAngle(serPort, roombaSpeed, turnAngle);
%turnAngle(serPort, roombaSpeed, turnAngle)
%Turns the Create by turnAngle degrees or the shortest equivalent degree. Positive turnAngle
%up to 180 degrees and negative turnAngle from -180 to -360 degrees turns the Create counterclockwise.
%Negative turnAngle up to -180 degrees and positive turnAngle from 180 to 360 degrees turns the Create
%clockwise. turnAngle > 360 or < -360 will be scaled to fit within +/- 360 degrees for optimal performance.
%turnAngle should be between +/- 360 degrees
%roombaSpeed should be between 0 and 0.2 m/s
% By; Joel Esposito, US Naval Academy, 2011

try
    
    %Flush Buffer
    N = serPort.BytesAvailable();
    while(N~=0)
        fread(serPort,N);
        N = serPort.BytesAvailable();
    end
    
    warning off
    global td
    
    if (roombaSpeed < 0) %Speed shouldn't be negative
        disp('WARNING: Speed inputted is negative. Should be positive. Taking the absolute value');
        roombaSpeed = abs(roombaSpeed);
    end
    
    if (abs(roombaSpeed) < .025) %Speed inputted is too low
        disp('WARNING: Speed inputted is too low. Setting speed to minimum, .025 m/s');
        roombaSpeed = .025;
    end
    
    if (turnAngle > 360 || turnAngle < -360) %sets range to +/- 360 degrees to avoid excess turning
        disp('Setting angle to be between +/- 360 degrees');
        if (turnAngle > 360)
            while (turnAngle > 360)
                turnAngle = turnAngle - 360;
            end
        else
            while (turnAngle < -360)
                turnAngle = turnAngle + 360;
            end
        end
    end
    
    if (turnAngle > 180) %Sets the degrees to the shortest path
        disp('Setting turn path to shortest route. Going to turn clockwise');
        turnAngle = turnAngle - 360;
    elseif (turnAngle < -180)
        disp('Setting turn path to shortest route. Going to turn counterclockwise');
        turnAngle = turnAngle + 360;
    end
    
    if (turnAngle < 0 ) % Makes sure the robot turns in the right direction
        turnDir = -eps;
    else
        turnDir = eps;
    end
    
    if turnAngle ~=0
        
        SetFwdVelRadiusRoomba(serPort, roombaSpeed, turnDir);
        fwrite(serPort, [157]);  fwrite(serPort,turnAngle, 'int16');
        pause(td)
        SetFwdVelRadiusRoomba(serPort, 0, 0);
        pause(td)
        fwrite(serPort, [154]);
        while( serPort.BytesAvailable() ==0)
            %disp('waiting to finish')
        end
        %disp('Done turnAngle')
        pause(td)
        
    end
catch
            disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
    end