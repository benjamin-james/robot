# robot
  a library for calculating the angles needed for a robot to move to a point, and for calculating the position of the end point.
# Summary
  This is a library of functions (two you might use) to calculate 
	a robot's position for reaching an object by changing one angle
	at a time to get closer and closer. One function does so by
	iterating until the margin of error is less than the error specified
	(<code>calculate_position</code>). The other function iterates until the specified
	number of iterations has been reached (<code>calculate_position_times</code>).
# Use
  To use the library, simply type
  <br><code>#include "robot.h"</code><br>
  into the file using the functions, and link with <code>robot.o</code> (the compiled library).
# Compiling
  Simply type <code>make</code> to build the library (and the demo!)
# Have fun!
