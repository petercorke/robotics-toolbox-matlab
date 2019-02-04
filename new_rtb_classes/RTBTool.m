classdef RTBTool < RTBRobotComponent
    %RTBTOOL RTB Representation of a robot tool
    %
    %   This is the place where a potentially new RTB robot classe will be
    %   born. At current stage it is a game of thoughts and place of
    %   conceptual design considerations. This is NOT productive code.
    
    % The RTBJoint class would implement the functionality of a generalized
    % end-effector. This could be a gripper, a wheel, a foot, a camera, a
    % welding device, whatever... It could have dynamic properties in
    % addition to some standardized properties. (dynamicprops inheritance).
    %
    properties
        Property1
    end
    
    methods
        function obj = RTBTool(inputArg1,inputArg2)
            %RTBTOOL Construct an instance of this class
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

