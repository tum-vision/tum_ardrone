#pragma once
/*
 * drone_autopilot:
 * This node controls the drone. requires drone_stateestimation.
 * 
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * 
 * and
 * 
 * Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the Workshop on Visual Control of Mobile Robots (ViCoMoR) at the IEEE/RJS International Conference on Intelligent Robot Systems (IROS), 2012. 
 *
 *
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
 
 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include "tum_ardrone/AutopilotParamsConfig.h"
#include "DroneController.h"
#include "std_msgs/Empty.h"
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
