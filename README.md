# Package tum_ardrone

This package contains the implementation corresponding to the following publications:

- [Scale-Aware Navigation of a Low-Cost Quadrocopter with a Monocular Camera](https://vision.in.tum.de/_media/spezial/bib/engel14ras.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Camera-Based Navigation of a Low-Cost Quadrocopter](https://vision.in.tum.de/_media/spezial/bib/engel12iros.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing](https://vision.in.tum.de/_media/spezial/bib/engel12vicomor.pdf) (J. Engel, J. Sturm, D. Cremers) 

You can find a [video](https://www.youtube.com/watch?feature=player_embedded&v=eznMokFQmpc) on *youtube*.
This Package builds on the well known monocular SLAM framework PTAM presented by Klein & Murray in their paper at ISMAR07. Please study the original PTAM website and the corresponding paper for more information on this part of the software. Also, be aware of the license that comes with it. 

The code works for both the AR.Drone 1.0 and 2.0, the default-parameters however are optimized for the AR.Drone 2.0 by now.

## Installation

### with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/tum-vision/tum_ardrone.git -b hydro-devel
cd ..
rosdep install tum_ardrone
catkin_make
```

## Quick start

#### Launch the nodes

``` bash
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone.launch
```

#### Check status

On the GUI, under Drone Communication Status, you should see:
- Drone Navdata: XHz (X > 100)
- Pose Estimates: 33Hz

#### Keyboard control

- focus drone_gui window
- press ESC to activate KB control
- fly around with KB (see [drone_gui](#drone_gui) for key assignments)

#### Joystick control

- rosrun joy joy_node
- press PS button on controller to activate it
- fly around (see [drone_gui](#drone_gui) for key assignments)

#### Autopilot

- type command "autoInit 500 800" in top-left text-field
- click Clear and Send (maybe click Reset first)
  => drone will takeoff & init PTAM, then hold position.
- click on video to interactively set target (relative to current position); see [drone_stateestimation](#drone_stateestimation)
  => first fly up 1m and then down 1m to facilitate a good scale estimate, dont start e.g. by flying horizontally over uneven terrain (!).
- always have a finger on ESC or on the joystick for emergency-keyboard control :)

## Nodes

### drone_stateestimation

Estimates the drone's position based on sent navdata, sent control commands and PTAM.

> IMPORTANT: requires messages to be sent on both /ardrone/navdata (>100Hz) and /ardrone/image_raw (>10Hz), i.e. a connected drone with running ardrone_autonomy node, or a .bag replay of at least those two channels. ardrone_autonomy should be started with:
``` bash
rosrun ardrone_autonomy ardrone_driver _navdata_demo:=0 _loop_rate:=500
```

#### Subscribed topics

- /ardrone/navdata
- /ardrone/image_raw
- /cmd_vel
- /tum_ardrone/com

#### Published topics

- /ardrone/predictedPose
- /tum_ardrone/com

#### Services

None

#### Parameters

- ~publishFreq: frequency, at which the drone's estimated position is calculated & published. Default: 30Hz
- ~calibFile: camera calibration file. If not set, the defaults are used (camcalib/ardroneX_default.txt).
- UseControlGains: whether to use control gains for EKF prediction.
- UsePTAM: whether to use PTAM pose estimates as EKF update
- UseNavdata: whether to use Navdata information for EKF update
> UsePTAM and UseNavdata are set to false, the EKF is never updated and acts as a pure simulator, prediciting the pose based on the control commands received (on /cmd_vel). Nice for experimenting.

- PTAMMapLock: lock PTAM map (no more KF)
- PTAMSyncLock: lock PTAM map sync (fix scale and pose offsets etc.)
- PTAMMaxKF: maximum amount of KF PTAM takes. 

- PTAMMinKFDist: min. distance between two KF (in meters)
- PTAMMinKFWiggleDist: min. distance between two KF (relative to mean scene depth).
- PTAMMinKFTimeDiff: min time between two KF.
> PTAM takes a new KF if (PTAMMinKFTimeDiff AND (PTAMMinKFDist OR PTAMMinKFWiggleDist)), and tracking is good etc.

- RescaleFixOrigin: If the scale of the Map is reestimated, only one point in the mapping PTAM <-> World remains fixed.
	If RescaleFixOrigin == false, this is the current pos. of the drone (to avoid sudden, large "jumps"). this however makes the map "drift".
	If RescaleFixOrigin == true, by default this is the initialization point where the second KF has been taken (drone pos. may jump suddenly, but map remains fixed.). The fixpoint may be set by the command "lockScaleFP".
                  
- c1 ... c8: prediction model parameters of the EKF. see "Camera-Based Navigation of a Low-Cost Quadrocopter"

#### Required tf transforms

TODO

#### Provided tf transforms

TODO

#### Using it

To properly estimate PTAM's scale, it is best to fly up and down a little bit (e.g. 1m up and 1m down) immediately after initialization.
There are two windows, one shows the video and PTAM's map points, the other one the map. To issue key commands, focus the respective window and hit a key. This generates a command on /tum_ardrone/com, which in turn is parsed and does something.

###### Video Window

![Video window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=video.png)

| Key   | /tum_adrone/com message | Action  |
|-------|-------------------------|---------|
| r     | "p reset"               | resets PTAM |
| u     | "p toggleUI"            | changes view |
| space | "p space"               | takes first / second Keyframe for PTAM's initialization |
| k     | "p keyframe"            | forces PTAM to take a keyframe |
| l     | "toggleLog"             | starts / stops extensive logging of all kinds of values to a file |
| m     | "p toggleLockMap"       | locks map, equivalent to parameter PTAMMapLock |
| n     | "p toggleLockSync"      | locks sync, equivalent to parameter PTAMSyncLock |

Clicking on the video window will generate waypoints, which are sent to drone_autopilot (if running):
- left-click: fly (x,y,0)m relative to the current position. image-center is (0,0), borders are 2m respectively.
- right-click: fly (0,0,y)m and rotate yaw by x degree. image-center is (0,0), borders are 2m and 90 degree respectively.

###### Map Window

![Map window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=map.png)

| Key   | /tum_adrone/com message | Action  |
|-------|-------------------------|---------|
| r     | "f reset"               | resets EKF and PTAM |
| u     | "m toggleUI"            | changes view |
| v     | "m resetView"           | resets viewpoint of viewer |
| l     | "toggleLog"             | starts / stops extensive logging of all kinds of values to a file |
| v     | "m clearTrail"          | clears green drone-trail |

### drone_autopilot

PID controller for the drone. Also includes basic way-point-following and automatic initialization. Requires [drone_stateestimation](#drone_stateestimation) to be running. The target is set via the /tum_ardrone/com topic.

#### Subscribed topics

- /ardrone/predictedPose

#### Published topics

- /cmd_vel
- /ardrone/land
- /ardrone/takeoff
- /ardrone/reset

#### Services

None

#### Parameters

- ~minPublishFreq: usually, a control command is sent for each pose estimate received from drone_stateestimation. However, if no pose estimate is received for more than minPublishFreq milliseconds, a HOVER command is sent, causing the drone to hover if for example drone_stateestimation is shut down suddenly. Default: 110.
- Ki_X, Kd_X, Kp_X: PID controller parameters for roll/pitch, gaz (up/down) and yaw.
- max_X: maximal respective control command sent (ever).
- rise_fac: rise commands are larger than respective drop commands by this factor. This is due to the drone sometimes dropping unpredictably fast for large drop commands, however rising somethimes requires large rise commands.
aggressiveness: multiplied to PI-component of all commands sent. Low values lead to the drone flying "slower".

#### Required tf transforms

TODO

#### Provided tf transforms

TODO

#### Using it

The behavior of the autopilot is set by sending commands on /tum_ardrone/com of the form "c COMMAND". A Queue of commands is kept, and as soon as one command is finished (for example a way point reached), the next command is popped. The queue can be cleared by sending "c clearCommands". Commands can be sent using the [drone_gui](#drone_gui) node. Some example scripts can be found in /flightPlans/*.txt. Possible commands are:

- autoInit [int moveTimeMS] [int waitTimeMS] [int riseTimeMs] [float initSpeed]

        takes control, starts drone, initializes PTAM. That is:
        - starts the drone & and waits riseTimeMs, the drone will rise to approx. 
          a height of 1m. 
        - initializes PTAM by taking the first KF, flying up (sending initSpeed as control command) for moveTimeMS, 
          waiting waitTimeMS and then taking the second KF. 
          This is done until success (flying up and down respectively).
        - good default values are "autoInit 500 800 4000 0.5" 

- autoTakeover [int moveTimeMS] [int waitTimeMS] [int riseTimeMs] [float initSpeed]

        takes control, initializes PTAM. The same as autoInit ..., 
        but to be used when the drone is already flying (skipps the first step).

- takeoff 

        - takes control, starts drone.
        - does not reset map or initialize PTAM

- setReference [doube x] [double y] [double z] [double yaw]

        sets reference point (used in goto X X X X).
  
- setMaxControl [double cap = 1.0]

        maximal control sent is capped at [cap], causing the drone to fly slower.
        
- setInitialReachDist [double dist = 0.2]

        drone has to come this close to a way point initially

- setStayWithinDist [double dist = 0.5]

        drone has to stay this close to a way point for a certain amount of time.
        
- setStayTime [double seconds = 2.0]

        time the drone has to stay within [stayWithinDist] for target to be reached.
        
- clearCommands

        clears all targets, such that the next command is executed immediately.

- goto [doube x] [double y] [double z] [double yaw]

        flies to position (x,y,z yaw), relative to current reference point.
        blocks until target is reached according to set parameters

- moveBy [doube x] [double y] [double z] [double yaw]
        
	moves by (x,y,z,yaw), relative to the current target position
        blocks until target is reached according to set parameters

- moveByRel [doube x] [double y] [double z] [double yaw]
        
	moves by (x,y,z,yaw), relative to the current estimated position 
        of the drone
        blocks until target is reached according to set parameters
        
- land
        
	initializes landing (use auto-land of drone)
        
- lockScaleFP
        
	sets the one point that does not change when the scale is re-estimated 
        to the current drone position. The scaleFP can only be set when PTAM is 
        good, i.e. this is delayed until PTAM is initialized and good.
        Need to set useWorldFixpoint in dynammic_reconfigure.

### drone_gui

This node offers a simple QT GUI to control the [drone_autopilot](#drone_autopilot) node, the [drone_stateestimation](#drone_stateestimation) node and fly the drone manually via keyboard or joystick.

#### Subscribed topics

- /cmd_vel
- /tum_ardrone/com
- /ardrone/takeoff
- /ardrone/land
- /ardrone/reset
- /ardrone/predictedPose
- /ardrone/navdata
- /joy

#### Published topics

- /cmd_vel
- /tum_ardrone/com
- /ardrone/takeoff
- /ardrone/land
- /ardrone/reset

#### Services

- calls /ardrone/togglecam
- calls /ardrone/flattrim

#### Parameters

None

#### Required tf transforms

None

#### Provided tf transforms

None

#### Using it

![Drone GUI](http://wiki.ros.org/tum_ardrone/drone_gui?action=AttachFile&do=get&target=ui.png)

###### Monitor Drone, Autopilot and Stateestimation Nodes (top-right part).

On the top-right, the current publish-frequency of important topics is displayed:
- Drone Navdata: monitors /ardrone/navdata. Should be around 150 to 200Hz with a connected drone, and running (and correctly configured) ardrone_autonomy node.
- Drone Control: monitors /cmd_vel. This is the frequency of how often control commands are published (and sent to the drone).
- Pose Estimates: monitors /ardrone/predictedPose. These are the state predictions generated by the drone_stateestimation node. By default, this should be 30Hz.
- Joy Input: monitors /joy. If you have a connected game-pad and running joy_node, this should be different from 0Hz.
- Pings (RTT): current wireless LAN latency (RTT in ms) for 500B and 20kB packages. If these are too high, reduce Wlan clutter.

###### Manual or joystick control of the drone.

The current control source has to be set (i.e. joystick or KB). The autopilot is only allowed to send control commands, if this is set to autopilot.

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

###### Autopilot Control

- write commands in top-left text field (or load eample from one of the files). You can simply add .txt files to flightplans/.
- click Send to transmit commands to the autopilot's command queue (automatically sets Control Source to Autopilot).
- click Clear to clear autopilot command queue and current target.
- click Reset to reset Autopilot, PTAM and EKF.

## Troubleshooting

- if drone doesnt start:
  - battery empty? (drone does not start if battery < ~18%)
  - Drone in emergency state? (if so, the four led's are red. click Emergency to reset).
- cannot control drone:
  - selected correct control source? maybe re-select.
- drone flies unstable using the autopilot:
  - adjust / reduce control parameters using dynamic_reconfigure (see readme_autopilot).
- drone broken: buy a new one.

## Known Bugs & Issues

- as the software was originally developed for the Ar.Drone 1.0, the pressure sensor and magnetometer are not used.
- drone_stateestimation crashes occasionally when PTAM init fails (crash occurs in PTAM code). 
  Happens in oarticular if there is no baseline in between the first two keyframes, hardly ever happens in praxis.

## Tips and Tricks

#### Camera calibration

- front, old drone: 0.672049, 0.899033, 0.502065, 0.513876, 0.271972
- front, new drone: 0.771557, 1.368560, 0.552779, 0.444056, 1.156010

calibrate with ethzasl_ptam.
to work with colored images, in src/CameraCalibrator.cc change:
- add #include <cv_bridge/cv_bridge.h>
- change function imageCallback(...) to
```cpp
void CameraCalibrator::imageCallback(const sensor_msgs::ImageConstPtr & img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	if(mCurrentImage.size().x != img->width || mCurrentImage.size().y != img->height)
		mCurrentImage.resize(CVD::ImageRef(img->width, img->height));

	memcpy(mCurrentImage.data(),cv_ptr->image.data,img->width * img->height);

	mNewImage = true;
}
```

#### Parameters: c1 to c8

can be estimated easily by 
- recording a flight: rosbag record -O calibFlight.bag /ardrone/image_raw /ardrone/navdata /cmd_vel
- playing back that flight: rosbag play -l calibFlight.bag
- starting two stateestimation nodes, one with remapped name and output:
  - rosrun tum_ardrone drone_stateestimation __name:=drone_stateestimationn2 /ardrone/predictedPose:=/ardrone/predictedPose2
  - rosrun tum_ardrone drone_stateestimation
- plotting the respective estimated values
  - e.g.: rxplot /ardrone/predictedPose/dx,/ardrone/predictedPose2/dx
- using dynamic reconfigure to 
  - in drone_stateestimation2, use only control gains
  - in drone_stateestimation, use NO control gains, but instead navdata / speeds / PTAM.
- setting c_i in drone_stateestimation2 such that graphs match best (play around).

#### Parameters: PID control

approximate in "simulation" based on c1 to c8:
- play back any record, to make stateestimation run (dont play /cmd_vel)
  - rosbag play -l calibFlightZ.bag --topics /ardrone/image_raw /ardrone/navdata
- run stateestimation, in dynamic reconfigure set only control updates to be used
- run gui and autopilot, and control
- plot control / respective pose e.g. with 
  - rxplot /ardrone/predictedPose/yaw /cmd_vel/angular/z

## Licence

The major part of this software package - that is everything except PTAM - is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html. PTAM (comprised of all files in /src/stateestimation/PTAM) has it's own licence, see http://www.robots.ox.ac.uk/~gk/PTAM/download.html. This licence in particular prohibits commercial use of the software.

