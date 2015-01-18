% open a connection to the VREP simulator
%
% Notes::
% - the VREP constructor needs to know where V-REP is installed.
% - This can come from the environment variable VREP
% - Or you can edit this file to something like
%    vrep=VREP('/Applications/V-REP_PRO_EDU_V3_1_3_rev2b_Mac');

%%begin

% create a connection to a running instance of V-REP
vrep = VREP();

% let's change our viewpoint, move the camera further away
%
% first we get an object to mirror the default camera
cam = vrep.camera('DefaultCamera');

% then get the camera position
p = cam.getpos

% then set the camera be twice as far away
cam.setpos(2*p)

% add the human figure Bill
bill = vrep.loadmodel('people/Walking Bill');

% get the initial pose of Bill
T = bill.getpose()

% now we can change his position
bill.setpos([0.1, 0.2, 0]);

% make him turn to his right
bill.setorient([0 0 -pi/4])

% make him lean forward a bit
bill.setorient([0 pi/8 0])

% now set him back to his initial position and orientation
bill.setpose(T);

% now we will setup some nested loops to make him shuffle around a square
orient = [0 0 0];
for j=1:4
    step = [0.05 0 0];
    bill.setorient(orient);
    % move forward
    for i=1:30
        % move relative to current pose
        bill.setpos(step, bill);
    end
    % turn at right angles
    orient(3) = orient(3) + pi/2;
end

% remove the model from the scene
bill.remove();

% now we will load a scene into VREP that contains Bill, a tree, a mirror
% and a camera
vrep.loadscene(which('scene1.ttt'), 'local');

% we will get a handle to the camera
camera = vrep.camera('Vision_sensor');

% now we start the simulator, it must be running in order for the
% camera to render

vrep.simstart();

% now we loop as the camera creeps forward (in the z-direction) and we
% display the frames
for i=1:20
    camera.setpos([0 0 0.05], camera);
    camera.grab();
    drawnow
end
         
% stop the simulation
vrep.simstop();

% clear the scene, remove all objects
vrep.clearscene();

% add a model of an ABB IRB140 robot
model = vrep.loadmodel('robots/non-mobile/ABB IRB 140');

% and create a VREP_arm object
arm = vrep.arm(model);

% set the pose of the robot's base
arm.setpose( transl(0.3, 0.4, 0));  

% get the joint angles
q = arm.getq();

% now move the robot's joints
arm.setq(q+0.3);
         
% clear the scene, remove all objects
vrep.clearscene();

% close the connection to the VREP simulator
clear vrep