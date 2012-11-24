Package tum_ardrone:

This package contains the implementation corresponding to the following papers: 
- Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
- Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing (J. Engel, J. Sturm, D. Cremers)
On my website, you can find a video of the AR.Drone 1.0 flying autonomously, using this package.


The package works for both the AR.Drone 1.0 and 2.0, the default-parameters however are optimized for the AR.Drone 2.0.

=== Quickstart: ===
1.
rosrun ardrone_autonomy ardrone_driver _navdata_demo:=0 _loop_rate:=500
rosrun tum_ardrone drone_stateestimation
rosrun tum_ardrone drone_autopilot
rosrun tum_ardrone drone_gui

2. 
check if it sais (under Drone Communication Status):
- Drone Navdata: XHz (X > 100)
- Pose Estimates: 33Hz

3a. KB control:
- focus drone_gui window
- press ESC to activate KB control
- fly around with KB (see readme_gui.txt for key assignments)

3b. Joystick control:
- rosrun joy joy_node
- press PS button on controller to activate it
- fly around (see readme_gui.txt for key assignments)

3c. Autopilot:
- type command "autoInit 500 800" in top-left text-field
- click Clear and Send (maybe click Reset first)
  => drone will takeoff & init PTAM, then hold position.
- click on video to interactively set target (relative to current position); see readme_stateestimation.txt.
  => first fly up 1m and then down 1m to facilitate a good scale estimate, dont start e.g. by flying horizontally over uneven terrain (!).
- always have a finger on ESC or on the joystick for emergency-keyboard control :)


=== Troubleshooting: ===
- if drone doesnt start:
  -> battery empty? (drone does not start if battery < ~18%)
  -> Drone in emergency state? (if so, the four led's are red. click Emergency to reset).
- cannot control drone:
  -> selected correct control source? maybe re-select.
- drone flies unstable using the autopilot:
  -> adjust / reduce control parameters using dynamic_reconfigure (see readme_autopilot).
- drone broken: buy a new one.

=== Nodes: ===
- drone_stateestimation: Drone Stateestimation, including PTAM & Visualization.
- drone_autopilot: Drone Controller, requires drone_stateestimation
- drone_gui: GUI for controlling the drone (with a Joystick or KB) and for controlling the autopilot and the stateestimation node.

=== Notes: ===
- the channel /tum_ardrone/com is used for communication between the nodes. to see what's going on internally, it is generally a good idea to echo this channel.


=== Known Bugs & Issues: ===
- as the software was originally developed for the Ar.Drone 1.0, the pressure sensor and magnetometer are not used.
- drone_stateestimation crashes occasionally when PTAM init fails (crash occurs in PTAM code). 
  Happens in oarticular if there is no baseline in between the first two keyframes, hardly ever happens in praxis.

