---------------------------------------------------------------------------------------------
Software 
---------------------------------------------------------------------------------------------
This version of the toolbox was developed using Octave 3.4.3 for windows but should still 
run on all distributions of octave. 

The latest version of octave can be found here: http://octave.sourceforge.net/


---------------------------------------------------------------------------------------------
Installation 
---------------------------------------------------------------------------------------------
To install the toolbox in octave the latest version of RVCTools is needed you can get this 
form http://www.petercorke.com/Robotics_Toolbox.html

Unzip the toolbox and place it in the m directory in your octave install (share\octave\3.4.3\m)  

Now copy all three folders from the Octave patch (@Quaternion @Link @SerialLink) to the 
directory (m\RVCtools\robot). Copy and replace all files and folders.

Once this is finished you are ready to use the toolbox in octave. Try >>> mdl_puma560 to 
build a robot. 

The code is a mashup of current V9 and the much older V6 code which supports Octave style classes. 


---------------------------------------------------------------------------------------------
Functionality of the toolbox 
---------------------------------------------------------------------------------------------
Most of the Toolbox should work as it does in MATLAB although there will be some functions 
that don't work correctly due to slight differences between MATLAB and Octave. 

One area of difference is the lack of reference objects in Octave.  Link and SerialLink objects 
inherit from the handle object in MATLAB which means that methods can change the value of the 
object.  This is not possible in Octave yet.  So in MATLAB you can write

obj.method(x)

but in Octave you have to write

obj = obj.method(x)

where method(x) changes some property of the object.


Quaternion:
-----------
.plot 		a little glitchy  

Link:
-----
Link seems to be working quite well 


SerialLink:
-----------
.plot 		function works but has some glitches
.payload 	only works when making a new robot e.g. p560 = p560.payload(2.5,[0 0 .1]) 
.teach 		is not implemented in this release 
.ikine 		is not working correctly and will most likely crash.
