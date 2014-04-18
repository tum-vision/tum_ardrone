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

#### Provided tf transforms

#### Using it

To properly estimate PTAM's scale, it is best to fly up and down a little bit (e.g. 1m up and 1m down) immediately after initialization.
There are two windows, one shows the video and PTAM's map points, the other one the map. To issue key commands, focus the respective window and hit a key. This generates a command on /tum_ardrone/com, which in turn is parsed and does something.

##### Video Window

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

##### Map Window

![Map window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=map.png)

| Key   | /tum_adrone/com message | Action  |
|-------|-------------------------|---------|
| r     | "f reset"               | resets EKF and PTAM |
| u     | "m toggleUI"            | changes view |
| v     | "m resetView"           | resets viewpoint of viewer |
| l     | "toggleLog"             | starts / stops extensive logging of all kinds of values to a file |
| v     | "m clearTrail"          | clears green drone-trail |

### drone_autopilot

### drone_gui

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

## Licence

The major part of this software package - that is everything except PTAM - is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html. PTAM (comprised of all files in /src/stateestimation/PTAM) has it's own licence, see http://www.robots.ox.ac.uk/~gk/PTAM/download.html. This licence in particular prohibits commercial use of the software.

