This package contains the implementation corresponding to the following publications:

- [Scale-Aware Navigation of a Low-Cost Quadrocopter with a Monocular Camera](https://vision.in.tum.de/_media/spezial/bib/engel14ras.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Camera-Based Navigation of a Low-Cost Quadrocopter](https://vision.in.tum.de/_media/spezial/bib/engel12iros.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing](https://vision.in.tum.de/_media/spezial/bib/engel12vicomor.pdf) (J. Engel, J. Sturm, D. Cremers) 

You can find a [video](https://www.youtube.com/watch?feature=player_embedded&v=eznMokFQmpc) on *youtube*.
This Package builds on the well known monocular SLAM framework PTAM presented by Klein & Murray in their paper at ISMAR07. Please study the original PTAM website and the corresponding paper for more information on this part of the software. Also, be aware of the license that comes with it. 

The code works for both the AR.Drone 1.0 and 2.0, the default-parameters however are optimized for the AR.Drone 2.0 by now.

# Installation

## with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/tum-vision/tum_ardrone.git -b hydro-devel
cd ..
rosdep install tum_ardrone
catkin_make
```

# Quick start

``` bash
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone.launch
```

# Nodes

## drone_stateestimation

Estimates the drone's position based on sent navdata, sent control commands and PTAM.
> IMPORTANT: requires messages to be sent on both /ardrone/navdata (>100Hz) and /ardrone/image_raw (>10Hz), i.e. a connected drone with running ardrone_autonomy node, or a .bag replay of at least those two channels. ardrone_autonomy should be started with:
``` bash
rosrun ardrone_autonomy ardrone_driver _navdata_demo:=0 _loop_rate:=500
```
### Subscribed topics

### Published topics

### Services

### Parameters

### Required tf transforms

### Provided tf transforms

## drone_autopilot

## drone_gui

# Licence

The major part of this software package - that is everything except PTAM - is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html. PTAM (comprised of all files in /src/stateestimation/PTAM) has it's own licence, see http://www.robots.ox.ac.uk/~gk/PTAM/download.html. This licence in particular prohibits commercial use of the software.

