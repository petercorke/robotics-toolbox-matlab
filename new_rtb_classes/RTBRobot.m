classdef RTBRobot < handle
    %RTBROBOT RTB Representation of a Robot
    %   This is the place where a potentially new RTB robot classe will be
    %   born. At current stage it is a game of thoughts and place of
    %   conceptual design considerations. This is NOT productive code.
    
    % RTBRobot: A handle class object 
    % A full robot, especially a legged one, can easily become abig data 
    % object. So it might not be a good idea to always pass deep copies
    % around. Next, robots are supposed to do things, interact with and
    % manipulate their environment. Deep copies can still be achieved if
    % really necessary by implementing an explicit 'copy' method.
    % So it might be a nice feature to exploit also the plenty of methods
    % that natively ship with the handles class, such as notifiers and
    % listeners.
    %
    % Other potential parent classes are:
    %   matlab.mixin.SetGet — Provides set and get methods to access 
    %   property values.
    %   dynamicprops — Enables you to define properties that are associated
    %   with an object, but not the class in general.
    %   matlab.mixin.Copyable Provides a copy method that you can customize
    %   for your class.
    %
    % The existing SerialLink objects can be modified to be a specialized
    % subclass of RTBRobot. It should be feasible to keep the function
    % interface consistent and backwards compatible while updating the
    % SerialLink implementation to make use of RTBRobot functionality. This
    % would enable backwards compatibility.
    % 
    
    properties
        graph       % graph representation of the physical robot
        name        % robot name (e.g. Wall-e)
        gravity     % gravity 
        base        % index of (floating) base node
        tool        % legacy tool transform
        manufacturer% name of the manufacturer
        comment     % some user information
        plotopt     % how to draw the robot
        fast        %
        interface   % SOME
        ikineType   %
        trail       %    MORE
        movie       %
        delay       %       ORIGINAL 
        loop        %
        model3d     %            SERIALLINK 
        faces       %
        points      %               CLASS
        plotopt3d   %                 
        n           %                      PROPERTIES
        links       % legacy... 
        T           % 
        config      %
        offset      % should go to joint objects? method for backwards compatibility
        qlim        % should go to joint objects? method for backwards compatibility
        d           %
        a           %
        alpha       %
        theta       %
    end
    
    methods
        function obj = RTBRobot(EdgeTable,NodeTable)
            %RTBROBOT Construct an instance of this class
            %   Detailed explanation goes here
            
            % The kinematic structure of the RTBRobot is represented as a
            % graph. The digraph class implemens a graph theory interface 
            % for directed graphs including loops. This would serve for the
            % definition of serial, tree-like and closed kinematic chains.
            % 
            % The EdgeTable and NodeTable option to create a graph could be
            % less efficient but human readable option to create the graph.
            % In particular because it permits the addition of arbitrary
            % variables and values to the edges and nodes. 
            %
            % Another option could be to keep a separate vector or cell of 
            % joint and link objects. The index within the vector or cell
            % would correspond to the index in the digraph.
            %
            % It might be a good idea to follow the approach in RBDL, where
            % links are nodes and joints are directed edges. This motivates
            % a generalized 'RTBLink' and a separate RTBJoint class, which
            % replace the original 'link' class. 
            %
            % RBDL is pretty fast and mature. Any implementation in MATLAB
            % will likely be slower. So it would be nice, to follow a dual 
            % approach with the original RTB functions and optional
            % wrappers for RBDL for those, who are willing to include the
            % additional code dependency.
            obj.graph = digraph(EdgeTable,NodeTable)
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        
        function clone = copy(obj)
            % COPY Creates an explicit deep copy of the robot.
            %   Detailed explanation goes here
    
    % The big amount of work is then to implement the already existing
    % functionality using the new data structure:
    % A              copy           genforces      ikunc          itorque        pay            rne_mdh        
    % DH             coriolis       getpos         inertia        jacob0         paycap         sym            
    % MDH            display        gravjac        isconfig       jacob_dot      payload        teach          
    % SerialLink     dyn            gravload       isdh           jacobe         perturb        todegrees      
    % accel          edit           ikcon          islimit        jacobn         plot           toradians      
    % addprop        fdyn           ikine          ismdh          jointdynamics  plot3d         trchain        
    % animate        fellipse       ikine3         isprismatic    jtraj          plus           twists         
    % char           fkine          ikine6s        isrevolute     maniplty       qmincon        vellipse       
    % cinertia       friction       ikine_sym      isspherical    mtimes         rne            
    % collisions     gencoords      ikinem         issym          nofriction     rne_dh         
    % 
    % Static methods:
    % 
    % URDF   
    end
end

