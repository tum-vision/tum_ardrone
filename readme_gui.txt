=== GENERAL: ===
Node: drone_gui.
This node offers a simple QT gui to control the autopilot node, the stateestimation node and the drone directly via keyboard or joystick. It has a couple of functionalities:

1. Monitor Drone, Autopilot and Stateestimation Nodes (top-right part).
It also regularely pings the drone, the measured pings are sent to the stateestimation node and used for data sync.


2. Manual or joystick control of the drone.
The current control source has to be set (i.e. joystick or KB). The autopilot is only allowed to send control commands, if this is set to autopilot (!).
- Joystick control requires a connected joystick and running "rosrun joy joy_node". We use a PS3 sixaxis controller.
  to make the controller work, a small linux-hack is required (set controller rights).
  - left joystick is horizontal pos. control; right joystick is height and yaw control.
  - L1 to take off, release L1 to land.
  - R1 to toggle emergency state.
  => by moving any of the two joysticks, the Control Source is immediately sent to Joystick.
     can be used for safety (autopilot does wired stuff -> immediately take over, disabling the autopilot and enabeling manual control).
- KB control requires the GUI window to have focus, but NOT the upper-left text field.
  => make the GUI the active window and press ESC to immediately enable KB control and set focus such that KB control works properly.
     can be used for safety (autopilot does wired stuff -> press ESC and immediately take over, disabling the autopilot and enabeling manual control).
  - q,a: fly up & down.
  - i,j,k,l: fly horizontally.
  - u,o: rotate yaw.
  - F1: toggle emergency
  - s: takeoff
  - d: land
- Buttons Land, Takeoff, ToggleCam, Emergency (toggles emergency state).


3. Autopilot Control:
- write commands in top-left text field (or load eample from one of the files). You can simply add .txt files to flightplans/.
- click Send to transmit commands to the autopilot's command queue (automatically sets Control Source to Autopilot).
- click Clear to clear autopilot command queue and current target.
- click Reset to reset Autopilot, PTAM and EKF.


=== PARAMETERS: ===
NONE

=== TOPICS: ===
reads & writes /cmd_vel
reads & writes /tum_ardrone/com
reads & writes /ardrone/takeoff
reads & writes /ardrone/land
reads & writes /ardrone/reset
reads /ardrone/predictedPose
reads /ardrone/navdata
reads /joy
calls /ardrone/togglecam
calls /ardrone/flattrim


=== DYNAMIC PARAMETERS: ===
NONE
