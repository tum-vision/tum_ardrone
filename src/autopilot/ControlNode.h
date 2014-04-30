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
#ifndef __CONTROLNODE_H
#define __CONTROLNODE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include "tum_ardrone/AutopilotParamsConfig.h"
#include "DroneController.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include "tum_ardrone/SetReference.h"
#include "tum_ardrone/SetMaxControl.h"
#include "tum_ardrone/SetInitialReachDistance.h"
#include "tum_ardrone/SetStayWithinDistance.h"
#include "tum_ardrone/SetStayTime.h"
#include "std_srvs/Empty.h"

class DroneKalmanFilter;
class MapView;
class PTAMWrapper;
class KIProcedure;


struct ControlNode
{
private:
	ros::Subscriber dronepose_sub;
	ros::Publisher vel_pub;
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher toggleState_pub;

	ros::NodeHandle nh_;
	static pthread_mutex_t tum_ardrone_CS;

	// parameters
	int minPublishFreq;
	std::string control_channel;
	std::string dronepose_channel;
	std::string command_channel;
	std::string packagePath;
	std::string land_channel;
	std::string takeoff_channel;
	std::string toggleState_channel;

	// services
	ros::ServiceServer setReference_;
	ros::ServiceServer setMaxControl_;
	ros::ServiceServer setInitialReachDistance_;
	ros::ServiceServer setStayWithinDistance_;
	ros::ServiceServer setStayTime_;
	ros::ServiceServer startControl_;
	ros::ServiceServer stopControl_;
	ros::ServiceServer clearCommands_;
	ros::ServiceServer hover_;
	ros::ServiceServer lockScaleFP_;

	bool setReference(tum_ardrone::SetReference::Request&, tum_ardrone::SetReference::Response&);
	bool setMaxControl(tum_ardrone::SetMaxControl::Request&, tum_ardrone::SetMaxControl::Response&);
	bool setInitialReachDistance(tum_ardrone::SetInitialReachDistance::Request&, tum_ardrone::SetInitialReachDistance::Response&);
	bool setStayWithinDistance(tum_ardrone::SetStayWithinDistance::Request&, tum_ardrone::SetStayWithinDistance::Response&);
	bool setStayTime(tum_ardrone::SetStayTime::Request&, tum_ardrone::SetStayTime::Response&);
	bool start(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool stop(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool clear(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool hover(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool lockScaleFP(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

	// command queue & KI stuff
	std::deque<std::string> commandQueue;
	static pthread_mutex_t commandQueue_CS;
	// this KI is currently responsible for setting the target etc.
	// if it is "Done", it is set to NULL,
	// if it is NULL, the next command will be popped and parsed from commandQueueu.
	KIProcedure* currentKI;

	// command parameters
	DronePosition parameter_referenceZero;
	double parameter_StayTime;
	double parameter_MaxControl;
	double parameter_InitialReachDist;
	double parameter_StayWithinDist;

	// functions
	void startControl();
	void stopControl();
	void clearCommands();
	void updateControl(const tum_ardrone::filter_stateConstPtr statePtr);

	void popNextCommand(const tum_ardrone::filter_stateConstPtr statePtr);
	void reSendInfo();
	char buf[500];
	ControlCommand lastSentControl;
public:
	ControlNode();
	~ControlNode();


	// ROS message callbacks
	void droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr);
	void comCb(const std_msgs::StringConstPtr str);
	void dynConfCb(tum_ardrone::AutopilotParamsConfig &config, uint32_t level);

	// main pose-estimation loop
	void Loop();

	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);

	// control drone functions
	void sendControlToDrone(ControlCommand cmd);
	void sendLand();
	void sendTakeoff();
	void sendToggleState();

	// controller
	DroneController controller;
	ControlCommand hoverCommand;

	// logging stuff
	std::ofstream* logfileControl;
	static pthread_mutex_t logControl_CS;
	void toogleLogging();	// switches logging on or off.

	// other internals
	long lastControlSentMS;
	bool isControlling;
};
#endif /* __CONTROLNODE_H */
