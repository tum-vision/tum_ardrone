#pragma once
 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ROSTHREAD_H
#define __ROSTHREAD_H
 
 

#include "cvd/thread.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/FlightAnim.h"
#include <std_msgs/Int16.h>


enum Animation_Type
{
    ARDRONE_ANIM_PHI_M30_DEG, ARDRONE_ANIM_PHI_30_DEG, ARDRONE_ANIM_THETA_M30_DEG, ARDRONE_ANIM_THETA_30_DEG,
    ARDRONE_ANIM_THETA_20DEG_YAW_200DEG, ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG, ARDRONE_ANIM_TURNAROUND,
    ARDRONE_ANIM_TURNAROUND_GODOWN, ARDRONE_ANIM_YAW_SHAKE, ARDRONE_ANIM_YAW_DANCE, ARDRONE_ANIM_PHI_DANCE,
    ARDRONE_ANIM_THETA_DANCE, ARDRONE_ANIM_VZ_DANCE, ARDRONE_ANIM_WAVE, ARDRONE_ANIM_PHI_THETA_MIXED,
    ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED, ARDRONE_ANIM_FLIP_AHEAD, ARDRONE_ANIM_FLIP_BEHIND, ARDRONE_ANIM_FLIP_LEFT,
    ARDRONE_ANIM_FLIP_RIGHT
};



class tum_ardrone_gui;

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};

class RosThread : private CVD::Thread
{
private:
	// the associated thread's run function.
	void run();

	// keep Running
	bool keepRunning;

	// ros stuff
	ros::Subscriber dronepose_sub;
	ros::Publisher vel_pub;
	ros::Subscriber vel_sub;
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;
	ros::Subscriber navdata_sub;
	ros::Subscriber joy_sub;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher toggleState_pub;
	ros::ServiceClient toggleCam_srv;
	std_srvs::Empty toggleCam_srv_srvs;
	ros::ServiceClient flattrim_srv;
	std_srvs::Empty flattrim_srv_srvs;
	ros::Subscriber takeoff_sub;
	ros::Subscriber land_sub;
	ros::Subscriber toggleState_sub;
    ros::ServiceClient animation_srv;
    ardrone_autonomy::FlightAnim animation_srv_srvs;
    ros::Subscriber leapMotion_sub;
    ros::Subscriber leapTakeoff_sub;
    ros::Subscriber leapLand_sub;
    ros::Subscriber leapGesture_sub;
    int joystick_type_;



	ros::NodeHandle nh_;

	// counters for Hz
	unsigned int dronePoseCount;
	unsigned int velCount;
	unsigned int navdataCount;
	unsigned int joyCount;
	unsigned int velCount100ms;

	static pthread_mutex_t send_CS;
public:
	RosThread(void);
	~RosThread(void);

	// start and stop system and respective thread.
	// to be called externally
	void startSystem();
	void stopSystem();

    static std::map<std::string,int> Animation_map;

    static std::map<std::string,int> create_map()
            {
              std::map<std::string,int> m;
              m["ARDRONE_ANIM_PHI_M30_DEG"] =  0;
              m["ARDRONE_ANIM_PHI_30_DEG"] =  1;
              m["ARDRONE_ANIM_THETA_M30_DEG"] =  2;
              m["ARDRONE_ANIM_THETA_30_DEG"] =  3;
              m["ARDRONE_ANIM_THETA_20DEG_YAW_200DEG"] =  4;
              m["ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG"] =  5;
              m["ARDRONE_ANIM_TURNAROUND"] =  6;
              m["ARDRONE_ANIM_TURNAROUND_GODOWN"] =  7;
              m["ARDRONE_ANIM_YAW_SHAKE"] =  8;
              m["ARDRONE_ANIM_YAW_DANCE"] =  9;
              m["ARDRONE_ANIM_PHI_DANCE"] =  10;
              m["ARDRONE_ANIM_THETA_DANCE"] =  11;
              m["ARDRONE_ANIM_VZ_DANCE"] =  12;
              m["ARDRONE_ANIM_WAVE"] =  13;
              m["ARDRONE_ANIM_PHI_THETA_MIXED"] =  14;
              m["ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED"] =  15;
              m["ARDRONE_ANIM_FLIP_AHEAD"] =  16;
              m["ARDRONE_ANIM_FLIP_BEHIND"] =  17;
              m["ARDRONE_ANIM_FLIP_LEFT"] =  18;
              m["ARDRONE_ANIM_FLIP_RIGHT"] =  19;
              return m;
            }

	tum_ardrone_gui* gui;


	// callbacks
	void droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr);
	void comCb(const std_msgs::StringConstPtr str);
	void velCb(const geometry_msgs::TwistConstPtr vel);
	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
	void joyCb(const sensor_msgs::JoyConstPtr joy_msg);
	void landCb(std_msgs::EmptyConstPtr);
	void toggleStateCb(std_msgs::EmptyConstPtr);
	void takeoffCb(std_msgs::EmptyConstPtr);
    void leapTakeoffCb(std_msgs::EmptyConstPtr);
    void leapLandCb(std_msgs::EmptyConstPtr);
    void leapMotionCb(geometry_msgs::TwistConstPtr leap_twist);
    void leapGestureCb(std_msgs::Int16 msg);


	ControlCommand lastJoyControlSent;
	bool lastL1Pressed;
	bool lastR1Pressed;


	// send command functions. can be called from any thread & are thread-safe.
	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);
	void sendControlToDrone(ControlCommand cmd);
	void sendLand();
	void sendTakeoff();
	void sendToggleState();
	void sendToggleCam();
	void sendFlattrim();
    void sendAnimation(int _type, uint32_t _duration=0);

};


#endif /* __ROSTHREAD_H */
