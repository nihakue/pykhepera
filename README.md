#pykhepera
=========

This is the python code for our Khepera robot. It's rad. 

##Usage
---------
Start an interactive python shell, import robot, and the create a
robot with r = robot.load_robot()

if you ever need to reload the robot, do just the same.


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

