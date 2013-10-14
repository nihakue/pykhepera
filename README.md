#pykhepera
=========

This is the python code for our Khepera robot. It's rad. 

##Usage
---------
Things are a bit of a mess right now. We're doing some refactoring, so there isn't a sure fired way to start things up.


##Commands
---------
###set_speed(speed)
sets the speed
###stop()
sets the speed to zero
###get_values(command)
sends the command to the robot, returns a list of the values
valid commands: 
+N: Proximity readings
+H: Counter Positions
+A: Read A/D value
+O: Ambient Light

