# robot
  a library for calculating the angles needed for a robot to move to a point, and for calculating the position of the end point.
# Summary
  This is a library of functions (three you might use) to calculate 
	a robot's position for reaching an object by changing one angle
	at a time to get closer and closer. One function does so by
	iterating until the margin of error is less than the error specified
	(<code>calculate_position</code>). Another function iterates until the specified
	number of iterations has been reached (<code>calculate_position_times</code>).
	The last one, (<code>moveTowards</code>), does the same as the previous function,
	but actually moves the angles according to the user-defined function <code>move</code>.
# Use
  To use the library, simply type
  <br><code>#include "robot.h"</code><br>
  into the file using the functions, and link with the dynamic library, see the output of <code>make install</code>.
# Compiling
  Follows autotools standard, first type <code>sh autogen.sh</code> to generate <code>./configure<code>
# Dependencies
  Needs autotools installed
# Have fun!
