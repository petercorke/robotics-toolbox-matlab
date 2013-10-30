% Brick Interface to Lego Minstorms EV3 brick
%
% Methods::
% brick             Constructor, establishes serial communications
% delete            Destructor, closes serial connection
% 
% beep              Plays a beep tone with volume and duration
% playTone          Plays a tone at a volume with a frequency and duration
% outputPower       Sets motor output power at a layer, NOS and speed
% outputStart       Starts motor at a layer, NOS and speed
% outputStop        Stops motor at a layer, NOS and brake
% uiReadLBatt       Returns battery level as a percentage
% uiReadVBatt       Returns battery level as a voltage
% outputGetCount    Returns the tachometer at a layer and NOS
% outputStepSpeed   Moves a motor to set position with layer, NOS, speed, ramp up angle, constant angle, ramp down angle and brake
% outputClrCount    Clears a motor tachometer at a  layer and NOS
% inputReadSI       Reads a connected sensor at a layer, NO, type and mode in SI units
% send              Send a serial packet
% receive           Return a received serial packet
%
% Example::
%           b = Brick('serConn','bt','btDevice','EV3','btChannel',1,'debug',1);
%           b = Brick('serConn','wifi','wfAddr','192.168.1.104','wfPort',5555,'wfSN','0016533dbaf5','debug',1);
% Notes::
% - layer is [0..3], should be 0
% - NOS is a bit field representing output 1 to 4 (0x01, 0x02, 0x04, 0x08)
% - speed is [+-0..100%]
% - brake is [0..1] (0=Coast,  1=Brake)
% - NO is [0..3] or the sensor port number minus 1

classdef Brick < handle
    
    % Communications format (c_com.h):
    %
    %   Direct Command Bytes:
    %   ,------,------,------,------,------,------,------,------,
    %   |Byte 0|Byte 1|Byte 2|Byte 3|Byte 4|Byte 5|      |Byte n|
    %   '------'------'------'------'------'------'------'------'
    % 
    %   Byte 0 – 1: Command size, Little Endian\n
    % 
    %   Byte 2 – 3: Message counter, Little Endian\n
    % 
    %   Byte 4:     Command type. see following defines   */
    % 
    %   #define     DIRECT_COMMAND_REPLY          0x00    //  Direct command, reply required
    %   #define     DIRECT_COMMAND_NO_REPLY       0x80    //  Direct command, reply not required
    % 
    %                                                     /*
    % 
    %   Byte 5 - 6: Number of global and local variables (compressed).
    % 
    %                Byte 6    Byte 5
    %               76543210  76543210
    %               --------  --------
    %               llllllgg  gggggggg
    % 
    %                     gg  gggggggg  Global variables [0..MAX_COMMAND_GLOBALS]
    % 
    %               llllll              Local variables  [0..MAX_COMMAND_LOCALS]
    % 
    %   Byte 7 - n: Byte codes
    %
    %                     Direct Command Response Bytes:
    %   ,------,------,------,------,------,------,------,------,
    %   |Byte 0|Byte 1|Byte 2|Byte 3|      |      |      |Byte n|
    %   '------'------'------'------'------'------'------'------'
    % 
    %   Byte 0 – 1: Reply size, Little Endian\n
    % 
    %   Byte 2 – 3: Message counter, Little Endian\n
    % 
    %   Byte 4:     Reply type. see following defines     */
    % 
    %   #define     DIRECT_REPLY                  0x02    //  Direct command reply
    %   #define     DIRECT_REPLY_ERROR            0x04    //  Direct command reply error
    % 
    %                                                     /*
    % 
    %   Byte 5 - n: Response buffer (global variable values)
    properties
        debug;
        serConn;
        btDevice;
        btChannel;
        wfAddr;
        wfPort; 
        wfSN; 
        serPort;
    end

    methods
        function brick = Brick(varargin) 
             % Brick.Brick Create a brick interface object
             %
             % b = Brick(OPTIONS) is an object that represents a connection
             % interface to a Lego Mindstorms EV3 brick
             %
             % Options::
             %  'debug',D       Debug level, show communications packet
             %  'serConn',P     Serial connection interface, either bt or wifi
             %  'btDevice',bt   Bluetooth brick device name
             %  'btChannel',cl  Bluetooth connection channel
             %  'wfAddr',wa     Wifi brick IP address
             %  'wfPort',pr     Wifi brick IP port, default 5555
             %  'wfSN',sn       Wifi brick serial number (found under Brick info on the brick OR through sniffing the UDP packets the brick emits on port 3015)
             %  'serPort',SP    Serial port connection object
             
             % init the properties
             opt.debug = false;
             opt.btDevice = [];
             opt.btChannel = 0;
             opt.wfAddr = '0.0.0.0';
             opt.wfPort = 5555;
             opt.wfSN = '0016533dbaf5';
             opt.serConn = '';
             % read in the options
             opt = tb_optparse(opt, varargin);
             % select the connection interface
             connect = 0;
             % bluetooth
             if(strcmp(opt.serConn,'bt'))
                brick.debug = opt.debug;
                brick.btDevice = opt.btDevice;
                brick.btChannel = opt.btChannel;
                brick.serPort = Bluetooth(brick.btDevice,brick.btChannel);
                fopen(brick.serPort);
                connect = 1;
             end
             % wifi
             if(strcmp(opt.serConn,'wifi'))
                brick.debug = opt.debug;
                brick.wfAddr = opt.wfAddr;
                brick.wfPort = opt.wfPort;
                brick.wfSN = opt.wfSN;
                brick.serPort = tcpip(brick.wfAddr,brick.wfPort);
                smsg = ['GET /target?sn=' brick.wfSN 'VMTP1.0' char(10) 'Protocol: EV3'];
                fopen(brick.serPort);
                fwrite(brick.serPort,smsg);
                pause(0.5);
                rmsg = fread(brick.serPort,16);
                if brick.debug > 0
                    fprintf('received:    %s\n',rmsg);
                end
                connect = 1;
             end
             % error
             if(~connect)
                 fprintf('Please specify a serConn option: ''bt'' or ''wifi''.\n');
             end
        end
        
        function delete(brick)
            % Brick.delete Close the serial connection
            %
            % delete(b) closes the serial connection
            
            fclose(brick.serPort);
        end  
        
        function s = char(brick)
            s = '';
        end
        
        function display(brick)
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(brick) );
        end
                
        function beep(brick, volume, duration)
            % Brick.beep Play beep tone
            %
            % Plays a beep tone with volume and duration in ms
            %
            % Example:: 
            %           b.beep(5,500)
            
            if nargin < 2
                volume = 10;
            end
            if nargin < 3
                duration = 100;
            end
            brick.playTone(volume, 1000, duration);
        end
        
        
        function playTone(brick, volume, freq, duration)  
            % Brick.beep Play Tone
            % Byte codes: opSOUND, LC0(SoundSubCodes), LC0(volume), LC4(freq), LC4(duration)
            % Plays a tone at a volume with a frequency in Hz and duration in ms
            %
            % Example:: 
            %           b.playTone(5,400,500)

            cmd = Command();
            cmd.addHeader(42,0,0);
            cmd.addOpCode(ByteCodes.Sound);
            cmd.LC0(SoundSubCodes.Tone);
            cmd.LC0(volume);
            cmd.LC4(freq);
            cmd.LC4(duration);
            cmd.addLength();
            brick.send(cmd);
        end
        
        % 
        function outputPower(brick,layer,nos,speed)
            % Brick.outputPower Set output power
            % Byte codes: opOUTPUT_POWER,LC0(layer),LC0(nos),LC1(speed)
            % Sets motor output power at a layer, NOS and speed.
            % 
            % Example::
            %           b.outputPower(0,Device.MotorA,50)
            
            cmd = Command();
            cmd.addHeader(42,0,0);
            cmd.addOpCode(ByteCodes.OutputPower);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
            cmd.addLength();
            brick.send(cmd);
        end
        
        function outputStart(brick,layer,nos)
            % Brick.outputStart Start a motor
            % Byte codes: opOUTPUT_START,LC0(layer),LC0(NOS)
            % Starts motor at a layer, NOS and speed
            %
            % Example::
            %           b.outputStart(0,Device.MotorA)
          
            cmd = Command();
            cmd.addHeader(42,0,0);
            cmd.addOpCode(ByteCodes.OutputStart);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.addLength();
            brick.send(cmd);
        end
        
        
        function outputStop(brick,layer,nos,brake)
            % Brick.outputPower Stop a motor
            % Byte codes: opOUTPUT_STOP,LC0(layer),LC0(NOS),LC0(brake)
            % Stops motor at a layer, NOS and brake
            %
            % Example::
            %           b.outputStop(0,Device.MotorA)
            
            cmd = Command();
            cmd.addHeader(42,0,0);
            cmd.addOpCode(ByteCodes.OutputStop);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC0(brake);
            cmd.addLength();
            brick.send(cmd);
        end
        
       
        function level = uiReadLbatt(brick)
            % Brick.uiReadLbatt Read battery level
            % Byte codes: opUIWrite,LC0(GetLbatt),GV0(0)
            % Returns battery level as a percentage
            %
            % Example::
            %           level = b.uiReadLbatt()
          
            cmd = Command();
            cmd.addHeaderReply(42,1,0);
            cmd.addOpCode(ByteCodes.UIRead);
            cmd.LC0(UIReadSubCodes.GetLbatt);
            cmd.GV0(0);
            cmd.addLength();
            brick.send(cmd);
            % receive the command
            msg = brick.receive()';
            level = msg(4);
            if brick.debug > 0
                fprintf('Battery level: %d%%\n', level);
            end
        end
        
        
        function voltage = uiReadVbatt(brick)
            % Brick.uiReadVbatt Return battery voltage
            % Byte codes: opUIWrite,LC0(GetVbatt),GV0(0)
            % Returns battery level as a voltage
            %
            % Example::
            %           voltage = b.uiReadVbatt()
            
            cmd = Command();
            cmd.addHeaderReply(42,4,0);
            cmd.addOpCode(ByteCodes.UIRead);
            cmd.LC0(UIReadSubCodes.GetVbatt);
            cmd.GV0(0);
            cmd.addLength();
            brick.send(cmd);
            % receive the command
            msg = brick.receive()';
            voltage = typecast(uint8(msg(4:7)),'single');           
            if brick.debug > 0
                fprintf('Battery voltage: %.02fV\n', voltage);
            end
        end
        
        function tacho = outputGetCount(brick,layer,nos)
            % Brick.outputGetCount Get tachometer count
            % Byte codes: opOUTPUT_GET_COUNT,LCO(layer),LC0(NOS),GV0(0)
            % Returns the tachometer at a layer and NOS
            %
            % Example::
            %           tacho = outputGetCount(0,Device.MotorA)

            cmd = Command();
            cmd.addHeaderReply(42,4,0);
            cmd.addOpCode(ByteCodes.OutputGetCount);
            cmd.LC0(layer);
            cmd.LC0(nos-1);
            cmd.GV0(0);
            cmd.addLength();
            brick.send(cmd);
            % receive the command
            msg = brick.receive()';
            tacho = typecast(uint8(msg(4:7)),'uint32');
            if brick.debug > 0
                fprintf('Tacho: %d degrees\n', tacho);
            end
        end
        

        function outputStepSpeed(brick,layer,nos,speed,step1,step2,step3,brake)
            % Brick.outputStepSpeed Output step speed
            % Byte codes: opOUTPUT_STEP_SPEED,LC0(layer),LC0(nos),LC1(speed),LC4(STEP1),LC4(STEP2),LC4(STEP3),LC0(BRAKE)
            % Moves a motor to set position with layer, NOS, speed, ramp up angle, constant angle, ramp down angle and brake
            %
            % Example::
            %           outputStepSpeed(0,Device.MotorA,50,50,360,50,Device.Coast)
            
            cmd = Command();
            cmd.addHeader(42,0,0);
            cmd.addOpCode(ByteCodes.OutputStepSpeed);
            cmd.LC0(layer);
            cmd.LC0(nos);
            cmd.LC1(speed);
            cmd.LC4(step1);
            cmd.LC4(step2);
            cmd.LC4(step3);
            cmd.LC0(brake);
            cmd.addLength();
            brick.send(cmd);
        end
        

        function outputClrCount(brick,layer,nos)
           % Brick.outputClrCount Clear tachometer
           % Byte codes: opOUTPUT_CLR_COUNT,LC0(layer),LC0(NOS)
           % Clears a motor tachometer at a layer and NOS
           %
           % Example::
           %            outputClrCount(0,Device.MotorA)
           
           cmd = Command();
           cmd.addHeader(42,0,0);
           cmd.addOpCode(ByteCodes.OutputClrCount);
           cmd.LC0(layer);
           cmd.LC0(nos);
           cmd.addLength();
           brick.send(cmd);
        end
        

        function reading = inputReadSI(brick,layer,no,type,mode)
           % Brick.inputReadSI Read sensor in SI units
           % Byte codes: opINPUT_READSI,LC0(layer),LC0(no),LC0(type),LC0(mode),GV0(0)
           % Reads a connected sensor at a layer, NO, type and mode in SI units
           %
           % Notes::
           % - type is the sensor type from types.html
           % - mode is the sensor mode from types.html
           %
           % Example::
           %            reading = inputReadSI(0,Device.Port1,Device.Ultraonsic,Device.USDistCM)
           %            reading = inputReadSI(0,Device.Port1,Device.Touch,Device.Pushed)
           
           cmd = Command();
           cmd.addHeaderReply(42,4,0);
           cmd.addOpCode(ByteCodes.InputReadSI);
           cmd.LC0(layer);
           cmd.LC0(no);
           cmd.LC0(type);
           cmd.LC0(mode);
           cmd.GV0(0);
           cmd.addLength();
           brick.send(cmd);
           % receive the command
           msg = brick.receive()';
           reading = typecast(uint8(msg(4:7)),'single');
           if brick.debug > 0
                fprintf('Sensor reading: %.02fV\n', reading);
           end
        end
        
        function send(brick, cmd)
           % Brick.send Send serial packet
           %
           % Send a serial packet over the serial port
           
           if isempty(brick.serPort)
               return
           end
           fwrite(brick.serPort, cmd.msg);
           if brick.debug > 0
               fprintf('sent:    [ ');
               for ii=1:length(cmd.msg)
                   fprintf('%d ',cmd.msg(ii))
               end
               fprintf(']\n');
           end
        end
       
        function rmsg = receive(brick)
           % Brick.receive Receive serial packet
           %
           % Return a received serial packet
 
           % get the reply size (little endian)
           nLength = fread(brick.serPort,2);
           % read the remaining bytes
           rmsg = fread(brick.serPort,double(typecast(uint8(nLength),'uint16')));
           if brick.debug > 0
               fprintf('received:    [ %d %d ', nLength(1),nLength(2));
               for ii=1:length(rmsg)
                   fprintf('%d ',rmsg(ii))
               end
               fprintf(']\n');
           end
        end
    end
end
