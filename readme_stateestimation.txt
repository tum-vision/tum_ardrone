=== GENERAL: ===
Node: drone_stateestimation.
Estimates the drone's position based on sent navdata, sent control commands and PTAM.
IMPORTANT: requires messages to be sent on both /ardrone/navdata (>100Hz) and /ardrone/image_raw (>10Hz), 
           i.e. a connected drone with running ardrone_autonomy node, or a .bag replay of at least those two channels.
           ardrone_autonomy should be started with:
           rosrun ardrone_autonomy ardrone_driver _navdata_demo:=0 _loop_rate:=500

To properly estimate PTAM's scale, it is best to fly up and down a little bit (e.g. 1m up and 1m down) immediately after initialization.
There are two windows, one shows the video and PTAM's map points, the other one the map. To issue key commands, focus the respective window and hit a key.
this generates a command on /tum_ardrone/com, which in turn is parsed and does something.

Video Window:
r -> "p reset": resets PTAM.
u -> "p toggleUI": changes view
space -> "p space": takes first / second Keyframe for PTAM's initialization
k -> "p keyframe": forces PTAM to take a keyframe.
l -> "toggleLog": starts / stops extensive logging of all kinds of values to a file.
m -> "p toggleLockMap": locks map, equivalent to "dyn. config parameter PTAMMapLock". may be overwritten by the dyn. config parameter setting.
n -> "p toggleLockSync": locks sync, equivalent to "dyn. config parameter PTAMSyncLock". may be overwritten by the dyn. config parameter setting.

Clicking on the video window will generate waypoints, which are sent to drone_autopilot (if running):
left-click: fly (x,y,0)m relative to the current position. image-center is (0,0), borders are 2m respectively.
right-click: fly (0,0,y)m and rotate yaw by x degree. image-center is (0,0), borders are 2m and 90 degree respectively.



Map Window:
r -> "f reset": resets EKF and PTAM.
u -> "m toggleUI": changes view
v -> "m resetView": resets viewpoint of viewer
l -> "toggleLog": starts / stops extensive logging of all kinds of values to a file.
v -> "m clearTrail": clears green drone-trail.



=== PARAMETERS: ===
~publishFreq: frequency, at which the drone's estimated position is calculated & published. Default: 30Hz
~calibFile: camera calibration file. If not set, the defaults are used (camcalib/ardroneX_default.txt).


=== TOPICS: ===
reads /ardrone/navdata
reads /ardrone/image_raw
reads /cmd_vel
writes /ardrone/predictedPose
reads & writes /tum_ardrone/com


=== DYNAMIC PARAMETERS: ===
UseControlGains: whether to use control gains for EKF prediction.
UsePTAM: whether to use PTAM pose estimates as EKF update
UseNavdata: whether to use Navdata information for EKF update
=> If UsePTAM and UseNavdata are set to false, the EKF is never updated and acts as a pure simulator, 
   prediciting the pose based on the control commands received (on /cmd_vel). Nice for experimenting.


PTAMMapLock: lock PTAM map (no more KF)
PTAMSyncLock: lock PTAM map sync (fix scale and pose offsets etc.)
PTAMMaxKF: maximum amount of KF PTAM takes. 


PTAMMinKFDist: min. distance between two KF (in meters)
PTAMMinKFWiggleDist: min. distance between two KF (relative to mean scene depth).
PTAMMinKFTimeDiff: min time between two KF.
=> PTAM takes a new KF if (PTAMMinKFTimeDiff AND (PTAMMinKFDist OR PTAMMinKFWiggleDist)), and tracking is good etc.


RescaleFixOrigin: If the scale of the Map is reestimated, only one point in the mapping PTAM <-> World remains fixed.
                  If RescaleFixOrigin == false, this is the current pos. of the drone (to avoid sudden, large "jumps"). this however makes the map "drift".
                  If RescaleFixOrigin == true, by default this is the initialization point where the second KF has been taken (drone pos. may jump suddenly, but map remains fixed.). The fixpoint may be set by the command "lockScaleFP".
                  
                  
c1 ... c8: prediction model parameters of the EKF. see "Camera-Based Navigation of a Low-Cost Quadrocopter"
