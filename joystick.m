%JOYSTICK Input from joystick
%
% J = JOYSTICK() returns a vector of joystick values in the range -1 to +1.
%
% [J,B] = JOYSTICK() as above but also returns a vector of button values, 
% either 0 (not pressed) or 1 (pressed).
%
% Notes::
% - The length of the vectors J and B depend on the capabilities of the
%   joystick identified when it is first opened.
