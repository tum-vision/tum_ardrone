#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 */
 

#include "cvd/thread.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"

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
};

