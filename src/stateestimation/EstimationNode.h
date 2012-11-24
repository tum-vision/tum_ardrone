#pragma once
/*
 * drone_stateestimation
 * This node publishes the drones estimated position, based on 
 * - drone nacdata
 * - PTAM
 * - sent control.
 *
 * This node does not send any control commands to the drone.
 *
 *
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include "tum_ardrone/StateestimationParamsConfig.h"


class DroneKalmanFilter;
class MapView;
class PTAMWrapper;

struct EstimationNode
{
private:
	// comm with drone
	ros::Subscriber navdata_sub; // drone navdata
	ros::Subscriber vel_sub; // to co-read contro commands sent from other thread
	ros::Subscriber vid_sub;
	ros::Time lastNavStamp;


	// comm with ptam
	//ros::Subscriber slam_info_sub; // ptam info (tracking quality) etc.
	//tf::TransformListener tf_sub;
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;
	static pthread_mutex_t tum_ardrone_CS;

	// output
	ros::Publisher dronepose_pub;

	ros::NodeHandle nh_;


	// parameters
	// every [publishFreq]ms, the node publishes the drones predicted position [predTime]ms into the future.
	// this pose can then be used to steer the drone. obviously, the larger [predTime], the worse the estimate.
	// this pose is published on /tf, and simultaneously further info is published on /ardrone/predictedPose
	ros::Duration predTime;
	int publishFreq;

	std::string navdata_channel;
	std::string control_channel;
	std::string output_channel;
	std::string video_channel;
	std::string command_channel;


	// for navdata time-smoothing
	long lastDroneTS;
	long lastRosTS;
	long droneRosTSOffset;


	// save last navinfo received for forwarding...
	ardrone_autonomy::Navdata lastNavdataReceived;

public:
	// filter
	DroneKalmanFilter* filter;
	PTAMWrapper* ptamWrapper;
	MapView* mapView;
	std::string packagePath;

	EstimationNode();
	~EstimationNode();


	// ROS message callbacks
	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
	void velCb(const geometry_msgs::TwistConstPtr velPtr);
	void vidCb(const sensor_msgs::ImageConstPtr img);
	void comCb(const std_msgs::StringConstPtr str);
	void dynConfCb(tum_ardrone::StateestimationParamsConfig &config, uint32_t level);

	// main pose-estimation loop
	void Loop();

	// writes a string message to "/tum_ardrone/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);
	void reSendInfo();


	// logging stuff
	// logging stuff
	std::ofstream* logfileIMU;
	std::ofstream* logfilePTAM;
	std::ofstream* logfileFilter;
	static pthread_mutex_t logIMU_CS;
	static pthread_mutex_t logPTAM_CS;
	static pthread_mutex_t logFilter_CS;
	long currentLogID;
	long startedLogClock;

	void toogleLogging();	// switches logging on or off.
	std::string calibFile;
	int arDroneVersion;


};
