classdef RTBLink < RTBRobotComponent
    %RTBLINK RTB Representation of a robot link.
    %
    %   This is the place where a potentially new RTB robot classes will be
    %   born. At current stage it is a game of thoughts and place of
    %   conceptual design considerations. This is NOT productive code.
    
    % The RTBJoint class could be a base class for the existing 'link' class. 
    % Though, here we might get a little into trouble because the original 
    % 'link' class does both jobs, the joint and the link job. This would
    % maybe the place to implement an optional  link (i.e. a handle or 
    % class name) to a joint class instance for backwards compatibility.
    %
    % Most robots are build from a few different link types. The RTBLink 
    % class would allow to associate a visual representation to a link
    % object and reuse this type of link in different ways and in
    % combinations with different joints.
    
    properties
        Property1
    end
    
    methods
        function obj = RTBLink(inputArg1,inputArg2)
            %RTBLINK Construct an instance of this class
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

