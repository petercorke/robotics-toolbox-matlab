%VREP_mirror V-REP mirror object class
%
% Mirror objects are MATLAB objects that reflect objects in the V-REP
% environment.  Methods allow the V-REP state to be examined or changed.
%
% This abstract class is the root class for all V-REP mirror objects.
%
% Methods throw exception if an error occurs.
%
% Methods::
%
%  setobjparam_bool    set object boolean parameter
%  setobjparam_int     set object integer parameter
%  setobjparam_float   set object float parameter
%
% See also VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.

classdef (Abstract=true) VREP_mirror < handle
    
    properties(GetAccess=public, SetAccess=protected)
        vrep
        h
        name
    end

            
        methods
            
            function obj = VREP_mirror(vrep, name)
            %VREP_mirror.VREP_mirror VREP_mirror object constructor
            %
            % v = VREP_mirror(NAME) creates a V-REP mirror object.
                obj.vrep = vrep;
                
                if nargin > 1
                     obj.h = vrep.gethandle(name);
                end
                obj.name = name;
            end
            
            function setobjparam_bool(obj, paramid, val)
                %VREP.setparam_bool Set boolean parameter of the V-REP simulator
                %
                % V.setparam_bool(NAME, VAL) sets the boolean parameter with name NAME
                % to value VAL within the V-REP simulation engine.
                obj.vrep.setobjparam_bool(obj, obj.h, paramid, val);
            end
            function setobjparam_int(obj, paramid, val)
                %VREP.setparam_bool Set integer parameter of the V-REP simulator
                %
                % V.setparam_int(NAME, VAL) sets the integer parameter with name NAME
                % to value VAL within the V-REP simulation engine.
                obj.vrep.setobjparam_int(obj, obj.h, paramid, val);
            end
            function setobjparam_float(obj, paramid, val)
                %VREP.setparam_float Set float parameter of the V-REP simulator
                %
                % V.setparam_float(NAME, VAL) sets the float parameter with name NAME
                % to value VAL within the V-REP simulation engine.
                obj.vrep.setobjparam_float(obj, obj.h, paramid, val);
            end
            
        end
end