classdef RTBRobotComponent
    %RTBROBOTCOMPONENT Abstract base class for robot components such as
    %joints, links and tools.
    %
    %   This is the place where a potentially new RTB robot classe will be
    %   born. At current stage it is a game of thoughts and place of
    %   conceptual design considerations. This is NOT productive code.
    
    properties
        Property1
    end
    
    methods
        function obj = RTBRobotComponent(inputArg1,inputArg2)
            %RTBROBOTCOMPONENT Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

