%UIDrawSubCodes UIDrawSubCodes enumeration for the brick
%
% Notes::
% - UIDrawSubCodes can be found in the EV3 documentation and source code
% (bytecodes.h)
% - USE SCREEN FOR GRAPHICS
%   -----------------------
%   
%     opUI_DRAW,LC0(FILLWINDOW),LC0(0),LC0(0),LC0(0)
%   
%       (clear entire screen by filling it with zero - background color)
%       
%       
%     opUI_DRAW, - - - - - - 
%     opUI_DRAW, - - - - - - 
%     opUI_DRAW, - - - - - - 
%     
%       (draw graphical stuff on the screen)
%       
%     
%     opUI_DRAW,LC0(UPDATE)  
%   
%       (show the stuff by updating the screen)
%   
% 
% 
%   RESTORE RUN SCREEN
%   ------------------
%   
%     opUI_WRITE,LC0(INIT_RUN)
% 
%       (enable the animated run screen)

classdef UIDrawSubCodes < uint8
    enumeration
         Update(0)
         Clean (1)
         Pixel (2)
         Line (3)
         Circle (4)
         Text (5)
         Icon (6)
         Picture (7)
         Value (8)
         Fillrect(9)
         Rect (10)
         Notification (11)
         Question (12)
         Keyboard (13)
         Browse (14)
         Vertbar (15)
         Inverserect (16)
         SelectFont (17)
         Topline (18)
         Fillwindow (19)
         Scroll (20)
         Dotline (21)
         ViewValue (22)
         ViewUnit (23)
         Fillcircle (24)
         Store (25)
         Restore (26)
         IconQuestion (27)
         Bmppfile(28)
         Popup (29)
         GraphSetup (30)
         GraphDraw (31)
         Textbox (32)
    end
end