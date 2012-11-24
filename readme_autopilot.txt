=== GENERAL: ===
Node: drone_autopilot.
PID controller for the drone. Also includes basic waypoint-following and automatic initialization.
requires drone_stateestimation to be running.
target is set via the /tum_ardrone/cmd topic.

=== PARAMETERS: ===
~minPublishFreq: usually, a control command is sent for each pose estimate received from drone_stateestimation.
                 However, if no pose estimate is received for more than minPublishFreq milliseconds, a HOVER command is sent,
                 causing the drone to hover if e.g. drone_stateestimation is shut down suddenly. Default: 110ms.
                 


=== TOPICS: ===
reads /ardrone/predictedPose
writes /cmd_vel
writes /ardrone/land
writes /ardrone/takeoff
writes /ardrone/reset


=== DYNAMIC PARAMETERS: ===
Ki_X, Kd_X, Kp_X: PIS controller parameters for roll/pitch, gaz (up/down) and yaw.
max_X: maximal respective control command sent (ever).
rise_fac: rise commands are larger than respective drop commands by this factor. This is due to the drone sometimes dropping unpredictably fast for large drop commands, however rising somethimes requires large rise commands.
agressiveness: multiplied to PI-component of all commands sent. low values lead to the drone flying "slower".


=== SCRIPTING LANGUAGE: ===
the behaviour of the autopilot is set by sending commands on /tum_ardrone/com of the form "c COMMAND". A Queue of commands is kept, and as soon as one command is finished (e.g. a waypoint reached), the next command is popped. The queue can be cleared by sending "c clearCommands".
possible commands are:

autoInit [int moveTimeMS] [int waitTimeMS]
	takes control, starts drone, inits PTAM. that is:
	- starts the drone & flies up a little bit. 
	- inits PTAM by taking the first KF, flying up for moveTimeMS, waiting waitTimeMS and then taking the second KF. this is done until success (flying up and down respectively).
	good default values are "autoInit 500 800" 

takeoff 
	- takes control, starts drone.
	- does not reset map or initialize PTAM

setReference [doube x] [double y] [double z] [double yaw]
  sets reference point (used in goto X X X X).
  
setMaxControl [double cap = 1.0]
	maximal control sent is capped at [cap], causing the drone to fly slower.
	
setInitialReachDist [double dist = 0.2]
	drone has to come this close to a waypoint initially

setStayWithinDist [double dist = 0.5]
	drone has to stay this close to a waypoint for a certain amount of time.
	
setStayTime [double seconds = 2.0]
	time the drone has to stay within [stayWithinDist] for target to be "reached"
	


clearCommands
	clears all targets, such that the next command is executed immediately.

goto [doube x] [double y] [double z] [double yaw]
	flies to position (x,y,z yaw), relative to current reference point.
	blocks until target is reached accoring to set parameters

moveBy [doube x] [double y] [double z] [double yaw]
	moves by (x,y,z,yaw), relative to the current target position
	blocks until target is reached accoring to set parameters

moveByRel [doube x] [double y] [double z] [double yaw]
	moves by (x,y,z,yaw), relative to the current estimated position of the drone
	blocks until target is reached accoring to set parameters
	
land
	initializes landing (use auto-land of drone)
	
lockScaleFP
	sets the one point that does not change when the scale is re-estimated to the current drone position.
	the scaleFP can only be set when PTAM is good, i.e. this is delayed until PTAM is initialized and good.
	need to set useWorldFixpoint in dynammic_reconfigure.

