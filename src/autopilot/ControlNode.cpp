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
 
 
 
#include "ControlNode.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/package.h>

#include "geometry_msgs/Twist.h"
#include "../HelperFunctions.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include <sys/stat.h>
#include <string>

// include KI's
#include "KI/KIAutoInit.h"
#include "KI/KIFlyTo.h"
#include "KI/KILand.h"
#include "KI/KIProcedure.h"

using namespace tum_ardrone;
using namespace std;

pthread_mutex_t ControlNode::logControl_CS = PTHREAD_MUTEX_INITIALIZER;


ControlNode::ControlNode()
{
    control_channel = nh_.resolveName("cmd_vel");
    dronepose_channel = nh_.resolveName("ardrone/predictedPose");
    command_channel = nh_.resolveName("tum_ardrone/com");
    takeoff_channel = nh_.resolveName("ardrone/takeoff");
    land_channel = nh_.resolveName("ardrone/land");
    toggleState_channel = nh_.resolveName("ardrone/reset");

	packagePath = ros::package::getPath("tum_ardrone");

	std::string val;
	float valFloat;

	ros::param::get("~minPublishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 110;
	minPublishFreq = valFloat;
	cout << "set minPublishFreq to " << valFloat << "ms"<< endl;


	// other internal vars
	logfileControl = 0;
	hoverCommand.gaz = hoverCommand.pitch = hoverCommand.roll = hoverCommand.yaw = 0;
	lastControlSentMS = 0;

	// channels
	dronepose_sub = nh_.subscribe(dronepose_channel, 10, &ControlNode::droneposeCb, this);
	vel_pub	   = nh_.advertise<geometry_msgs::Twist>(control_channel,1);
	tum_ardrone_pub	   = nh_.advertise<std_msgs::String>(command_channel,50);
	tum_ardrone_sub	   = nh_.subscribe(command_channel,50, &ControlNode::comCb, this);
	takeoff_pub	   = nh_.advertise<std_msgs::Empty>(takeoff_channel,1);
	land_pub	   = nh_.advertise<std_msgs::Empty>(land_channel,1);
	toggleState_pub	   = nh_.advertise<std_msgs::Empty>(toggleState_channel,1);

	// services handler
	setReference_ = nh_.advertiseService("drone_autopilot/setReference", &ControlNode::setReference, this);
	setMaxControl_ = nh_.advertiseService("drone_autopilot/setMaxControl", &ControlNode::setMaxControl, this);
	setInitialReachDistance_ = nh_.advertiseService("drone_autopilot/setInitialReachDistance", &ControlNode::setInitialReachDistance, this);
	setStayWithinDistance_ = nh_.advertiseService("drone_autopilot/setStayWithinDistance", &ControlNode::setStayWithinDistance, this);
	setStayTime_ = nh_.advertiseService("drone_autopilot/setStayTime", &ControlNode::setStayTime, this);
	startControl_ = nh_.advertiseService("drone_autopilot/start", &ControlNode::start, this);
	stopControl_ = nh_.advertiseService("drone_autopilot/stop", &ControlNode::stop, this);
	clearCommands_ = nh_.advertiseService("drone_autopilot/clearCommands", &ControlNode::clear, this);
	hover_ = nh_.advertiseService("drone_autopilot/hover", &ControlNode::hover, this);
	lockScaleFP_ = nh_.advertiseService("drone_autopilot/lockScaleFP", &ControlNode::lockScaleFP, this);

	// internals
	parameter_referenceZero = DronePosition(TooN::makeVector(0,0,0),0);
	parameter_MaxControl = 1;
	parameter_InitialReachDist = 0.2;
	parameter_StayWithinDist = 0.5;
	parameter_StayTime = 2;
	isControlling = false;
	currentKI = NULL;
	lastSentControl = ControlCommand(0,0,0,0);

	// create controller
	controller = DroneController();
	controller.node = this;
}

ControlNode::~ControlNode()
{

}

pthread_mutex_t ControlNode::commandQueue_CS = PTHREAD_MUTEX_INITIALIZER;
void ControlNode::droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr)
{
	// do controlling
	pthread_mutex_lock(&commandQueue_CS);

	// as long as no KI present:
	// pop next KI (if next KI present).
	while(currentKI == NULL && commandQueue.size() > 0)
		popNextCommand(statePtr);

	// if there is no current KI now, we obviously have no current goal -> send drone hover
	if(currentKI != NULL)
	{
		// let current KI control.
		this->updateControl(statePtr);
	}
	else if(isControlling)
	{
		sendControlToDrone(hoverCommand);
		ROS_DEBUG("Autopilot is Controlling, but there is no KI -> sending HOVER");
	}


	pthread_mutex_unlock(&commandQueue_CS);
}

// pops next command(s) from queue (until one is found thats not "done" yet).
// assumes propery of command queue lock exists (!)
void ControlNode::popNextCommand(const tum_ardrone::filter_stateConstPtr statePtr)
{
	// should actually not happen., but to make shure:
	// delete existing KI.
	if(currentKI != NULL)
	{
		delete currentKI;
		currentKI = NULL;
	}

	// read next command.
	while(currentKI == NULL && commandQueue.size() > 0)
	{
		std::string command = commandQueue.front();
		commandQueue.pop_front();
		bool commandUnderstood = false;

		// print me
		ROS_INFO("executing command: %s",command.c_str());

		int p;
		char buf[100];
		float parameters[10];

		// replace macros
		if((p = command.find("$POSE$")) != std::string::npos)
		{
			snprintf(buf,100, "%.3f %.3f %.3f %.3f",statePtr->x,statePtr->y,statePtr->z,statePtr->yaw);
			command.replace(p,6,buf);
		}
		if((p = command.find("$REFERENCE$")) != std::string::npos)
		{
			snprintf(buf,100, "%.3f %.3f %.3f %.3f",parameter_referenceZero.pos[0],parameter_referenceZero.pos[1],parameter_referenceZero.pos[2],parameter_referenceZero.yaw);
			command.replace(p,11,buf);
		}

		// -------- commands -----------
		// autoInit
		if(sscanf(command.c_str(),"autoInit %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIAutoInit(true,parameters[0],parameters[1],parameters[2],parameters[3],true);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		else if(sscanf(command.c_str(),"autoTakeover %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIAutoInit(true,parameters[0],parameters[1],parameters[2],parameters[3],false);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		// takeoff
		else if(command == "takeoff")
		{
			currentKI = new KIAutoInit(false);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		// setOffset
		else if(sscanf(command.c_str(),"setReference %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			parameter_referenceZero = DronePosition(TooN::makeVector(parameters[0],parameters[1],parameters[2]),parameters[3]);
			commandUnderstood = true;
		}

		// setMaxControl
		else if(sscanf(command.c_str(),"setMaxControl %f",&parameters[0]) == 1)
		{
			parameter_MaxControl = parameters[0];
			commandUnderstood = true;
		}

		// setInitialReachDist
		else if(sscanf(command.c_str(),"setInitialReachDist %f",&parameters[0]) == 1)
		{
			parameter_InitialReachDist = parameters[0];
			commandUnderstood = true;
		}

		// setStayWithinDist
		else if(sscanf(command.c_str(),"setStayWithinDist %f",&parameters[0]) == 1)
		{
			parameter_StayWithinDist = parameters[0];
			commandUnderstood = true;
		}

		// setStayTime
		else if(sscanf(command.c_str(),"setStayTime %f",&parameters[0]) == 1)
		{
			parameter_StayTime = parameters[0];
			commandUnderstood = true;
		}

		// goto
		else if(sscanf(command.c_str(),"goto %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIFlyTo(
				DronePosition(
				TooN::makeVector(parameters[0],parameters[1],parameters[2]) + parameter_referenceZero.pos,
					parameters[3] + parameter_referenceZero.yaw),
				parameter_StayTime,
				parameter_MaxControl,
				parameter_InitialReachDist,
				parameter_StayWithinDist
				);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;

		}

		// moveBy
		else if(sscanf(command.c_str(),"moveBy %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIFlyTo(
				DronePosition(
				TooN::makeVector(parameters[0],parameters[1],parameters[2]) + controller.getCurrentTarget().pos,
					parameters[3] + controller.getCurrentTarget().yaw),
				parameter_StayTime,
				parameter_MaxControl,
				parameter_InitialReachDist,
				parameter_StayWithinDist
				);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;

		}

		// moveByRel
		else if(sscanf(command.c_str(),"moveByRel %f %f %f %f",&parameters[0], &parameters[1], &parameters[2], &parameters[3]) == 4)
		{
			currentKI = new KIFlyTo(
				DronePosition(
				TooN::makeVector(parameters[0]+statePtr->x,parameters[1]+statePtr->y,parameters[2]+statePtr->z),
					parameters[3] + statePtr->yaw),
				parameter_StayTime,
				parameter_MaxControl,
				parameter_InitialReachDist,
				parameter_StayWithinDist
				);
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;

		}

		// land
		else if(command == "land")
		{
			currentKI = new KILand();
			currentKI->setPointers(this,&controller);
			commandUnderstood = true;
		}

		// setScaleFP
		else if(command == "lockScaleFP")
		{
			publishCommand("p lockScaleFP");
			commandUnderstood = true;
		}

		if(!commandUnderstood)
			ROS_INFO("unknown command, skipping!");
	}

}

void ControlNode::comCb(const std_msgs::StringConstPtr str)
{
	// only handle commands with prefix c
	if(str->data.length() > 2 && str->data.substr(0,2) == "c ")
	{
		std::string cmd =str->data.substr(2,str->data.length()-2);

		if(cmd.length() == 4 && cmd.substr(0,4) == "stop")
		{
			stopControl();
		}
		else if(cmd.length() == 5 && cmd.substr(0,5) == "start")
		{
			startControl();
		}
		else if(cmd.length() == 13 && cmd.substr(0,13) == "clearCommands")
		{
			clearCommands();
		}
		else
		{
			pthread_mutex_lock(&commandQueue_CS);
			commandQueue.push_back(cmd);
			pthread_mutex_unlock(&commandQueue_CS);
		}
	}

	// global command: toggle log
	if(str->data.length() == 9 && str->data.substr(0,9) == "toggleLog")
	{
		this->toogleLogging();
	}
}

void ControlNode::Loop()
{
	ros::Time last = ros::Time::now();
	ros::Time lastStateUpdate = ros::Time::now();

	while (nh_.ok())
	{

		// -------------- 1. spin for 50ms, do main controlling part here. ---------------
		while((ros::Time::now() - last) < ros::Duration(minPublishFreq / 1000.0))
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(minPublishFreq / 1000.0 - (ros::Time::now() - last).toSec()));
		last = ros::Time::now();


		// -------------- 2. send hover (maybe). ---------------
		if(isControlling && getMS(ros::Time::now()) - lastControlSentMS > minPublishFreq)
		{
			sendControlToDrone(hoverCommand);
			ROS_WARN("Autopilot enabled, but no estimated pose received - sending HOVER.");
		}

		// -------------- 2. update info. ---------------
		if((ros::Time::now() - lastStateUpdate) > ros::Duration(0.4))
		{
			reSendInfo();
			lastStateUpdate = ros::Time::now();
		}
	}
}
void ControlNode::dynConfCb(tum_ardrone::AutopilotParamsConfig &config, uint32_t level)
{
	controller.Ki_gaz = config.Ki_gaz;
	controller.Kd_gaz = config.Kd_gaz;
	controller.Kp_gaz = config.Kp_gaz;

	controller.Ki_rp = config.Ki_rp;
	controller.Kd_rp = config.Kd_rp;
	controller.Kp_rp = config.Kp_rp;

	controller.Ki_yaw = config.Ki_yaw;
	controller.Kd_yaw = config.Kd_yaw;
	controller.Kp_yaw = config.Kp_yaw;

	controller.max_gaz_drop = config.max_gaz_drop;
	controller.max_gaz_rise = config.max_gaz_rise;
	controller.max_rp = config.max_rp;
	controller.max_yaw = config.max_yaw;
	controller.agressiveness = config.agressiveness;
	controller.rise_fac = config.rise_fac;
}

pthread_mutex_t ControlNode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
void ControlNode::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&tum_ardrone_CS);
	tum_ardrone_pub.publish(s);
	pthread_mutex_unlock(&tum_ardrone_CS);
}


void ControlNode::toogleLogging()
{
	// logging has yet to be integrated.
}

void ControlNode::sendControlToDrone(ControlCommand cmd)
{
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	// assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
	cmdT.angular.x = cmdT.angular.y = 0;

	if(isControlling)
	{
		vel_pub.publish(cmdT);
		lastSentControl = cmd;
	}

	lastControlSentMS = getMS(ros::Time::now());
}

void ControlNode::sendLand()
{
	if(isControlling)
		land_pub.publish(std_msgs::Empty());
}
void ControlNode::sendTakeoff()
{
	if(isControlling)
		takeoff_pub.publish(std_msgs::Empty());
}
void ControlNode::sendToggleState()
{
	if(isControlling)
		toggleState_pub.publish(std_msgs::Empty());
}
void ControlNode::reSendInfo()
{
	/*
	Idle / Controlling (Queue: X)
	Current:
	Next:
	Target: X,X,X,X
	Error: X,X,X,X
	*/

	DronePosition p = controller.getCurrentTarget();
	TooN::Vector<4> e = controller.getLastErr();
	double ea = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
	snprintf(buf,500,"u c %s (Queue: %d)\nCurrent: %s\nNext: %s\nTarget: (%.2f,  %.2f,  %.2f), %.1f\nError: (%.2f,  %.2f,  %.2f), %.1f (|.| %.2f)\nCont.: r %.2f, p %.2f, g %.2f, y %.2f",
			isControlling ? "Controlling" : "Idle",
			(int)commandQueue.size(),
			currentKI == NULL ? "NULL" : currentKI->command.c_str(),
			commandQueue.size() > 0 ? commandQueue.front().c_str() : "NULL",
			p.pos[0],p.pos[1],p.pos[2],p.yaw,
			e[0],e[1],e[2],e[3], ea,
			lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw);

	publishCommand(buf);
}

void ControlNode::startControl() {
	isControlling = true;
	publishCommand("u l Autopilot: Start Controlling");
	ROS_INFO("START CONTROLLING!");
}

void ControlNode::stopControl() {
	isControlling = false;
	publishCommand("u l Autopilot: Stop Controlling");
	ROS_INFO("STOP CONTROLLING!");
}

void ControlNode::updateControl(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (currentKI->update(statePtr) && commandQueue.size() > 0) {
		delete currentKI;
		currentKI = NULL;
	}
}

void ControlNode::clearCommands() {
	pthread_mutex_lock(&commandQueue_CS);
	commandQueue.clear();						// clear command queue.
	controller.clearTarget();					// clear current controller target
	if(currentKI != NULL) delete currentKI;	// destroy & delete KI.
	currentKI = NULL;
	pthread_mutex_unlock(&commandQueue_CS);
	publishCommand("u l Autopilot: Cleared Command Queue");
	ROS_INFO("Cleared Command Queue!");
}

bool ControlNode::setReference(SetReference::Request& req, SetReference::Response& res)
{
	ROS_INFO("calling service setReference");
	parameter_referenceZero = DronePosition(TooN::makeVector(req.x, req.y, req.z), req.heading);	
	res.status = true;
	return true;
}

bool ControlNode::setMaxControl(SetMaxControl::Request& req, SetMaxControl::Response& res)
{
	ROS_INFO("calling service setMaxControl");
	parameter_MaxControl = req.speed;
	res.status = true;
	return true;
}

bool ControlNode::setInitialReachDistance(SetInitialReachDistance::Request& req, SetInitialReachDistance::Response& res)
{
	ROS_INFO("calling service setInitialReachDistance");
	parameter_InitialReachDist = req.distance;
	res.status = true;
	return true;
}

bool ControlNode::setStayWithinDistance(SetStayWithinDistance::Request& req, SetStayWithinDistance::Response& res) {
	ROS_INFO("calling service setStayWithinDistance");
	parameter_StayWithinDist = req.distance;
	res.status = true;
	return true;
}

bool ControlNode::setStayTime(SetStayTime::Request& req, SetStayTime::Response& res) {
	ROS_INFO("calling service setStayTime");
	parameter_StayTime = req.duration;
	res.status = true;
	return true;
}

bool ControlNode::start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	ROS_INFO("calling service start");
	this->startControl();
	return true;
}

bool ControlNode::stop(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ROS_INFO("calling service stop");
	this->stopControl();
	return true;
}

bool ControlNode::clear(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ROS_INFO("calling service clearCommands");
	this->clearCommands();
	return true;
}

bool ControlNode::hover(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ROS_INFO("calling service hover");
	this->sendControlToDrone(hoverCommand);
	return true;
}

bool ControlNode::lockScaleFP(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	ROS_INFO("calling service lockScaleFP");
	this->publishCommand("p lockScaleFP");
	return true;
}

