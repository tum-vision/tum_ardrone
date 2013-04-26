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
 
 
#include "EstimationNode.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "../HelperFunctions.h"
#include "DroneKalmanFilter.h"
#include <ardrone_autonomy/Navdata.h>
#include "deque"
#include "tum_ardrone/filter_state.h"
#include "PTAMWrapper.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "MapView.h"
#include <sys/stat.h>
#include <string>

using namespace std;

pthread_mutex_t EstimationNode::logIMU_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EstimationNode::logPTAM_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EstimationNode::logFilter_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EstimationNode::logPTAMRaw_CS = PTHREAD_MUTEX_INITIALIZER;

EstimationNode::EstimationNode()
{
    navdata_channel = nh_.resolveName("ardrone/navdata");
    control_channel = nh_.resolveName("cmd_vel");
    output_channel = nh_.resolveName("ardrone/predictedPose");
    video_channel = nh_.resolveName("ardrone/image_raw");
    command_channel = nh_.resolveName("tum_ardrone/com");
	packagePath = ros::package::getPath("tum_ardrone");

	std::string val;
	float valFloat = 0;

	predTime = ros::Duration(25*0.001);

	ros::param::get("~publishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 30;
	publishFreq = valFloat;
	cout << "set publishFreq to " << valFloat << "Hz"<< endl;



	ros::param::get("~calibFile", calibFile);
	if(calibFile.size()>0)
		cout << "set calibFile to " << calibFile << endl;
	else
		cout << "set calibFile to DEFAULT" << endl;


	navdata_sub       = nh_.subscribe(navdata_channel, 10, &EstimationNode::navdataCb, this);
	vel_sub          = nh_.subscribe(control_channel,10, &EstimationNode::velCb, this);
	vid_sub          = nh_.subscribe(video_channel,10, &EstimationNode::vidCb, this);

	dronepose_pub	   = nh_.advertise<tum_ardrone::filter_state>(output_channel,1);

	tum_ardrone_pub	   = nh_.advertise<std_msgs::String>(command_channel,50);
	tum_ardrone_sub	   = nh_.subscribe(command_channel,50, &EstimationNode::comCb, this);

	//tf_broadcaster();

	// other internal vars
	logfileIMU = logfilePTAM = logfileFilter = logfilePTAMRaw = 0;
	currentLogID = 0;
	lastDroneTS = 0;
	lastRosTS = 0;
	droneRosTSOffset = 0;
	lastNavStamp = ros::Time(0);
	filter = new DroneKalmanFilter(this);
	ptamWrapper = new PTAMWrapper(filter, this);
	mapView = new MapView(filter, ptamWrapper, this);
	arDroneVersion = 0;
	//memset(&lastNavdataReceived,0,sizeof(ardrone_autonomy::Navdata));


}

EstimationNode::~EstimationNode()
{
	filter->release();
	delete mapView;
	delete ptamWrapper;
	delete filter;


	//delete infoQueue;
}
void EstimationNode::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	lastNavdataReceived = *navdataPtr;
	if(ros::Time::now() - lastNavdataReceived.header.stamp > ros::Duration(30.0))
		lastNavdataReceived.header.stamp = ros::Time::now();

	if(arDroneVersion == 0)
	{
		arDroneVersion = (navdataPtr->pressure == 0) ? 1 : 2;
		std::cout <<"Found ARDrone Version " << arDroneVersion << std::endl;
	}


	// darn ROS really messes up timestamps.
	// they should arrive every 5ms, with occasionally dropped packages.
	// instead, they arrive with gaps of up to 30ms, and then 6 packages with the same timestamp.
	// so: this procedure "smoothes out" received package timestamps, shifting their timestamp by max. 20ms to better fit the order.
	long rosTS = getMS(lastNavdataReceived.header.stamp);
	long droneTS = navdataPtr->tm / 1000;

	if(lastDroneTS == 0) lastDroneTS = droneTS;

	if((droneTS+1000000) < lastDroneTS)
	{
		droneRosTSOffset = rosTS - droneTS;	// timestamp-overflow, reset running average.
		ROS_WARN("Drone Navdata timestamp overflow! (should happen epprox every 30min, while drone switched on)");
	}
	else
		droneRosTSOffset = 0.9 * droneRosTSOffset + 0.1*(rosTS - droneTS);

	long rosTSNew =droneTS + droneRosTSOffset;	// this should be the correct timestamp.
	long TSDiff = std::min(100l,std::max(-100l,rosTSNew-rosTS));	// never change by more than 100ms.
	lastNavdataReceived.header.stamp += ros::Duration(TSDiff/1000.0);	// change!
	lastRosTS = rosTS;
	lastDroneTS = droneTS;


	// convert to originally sent drone values (undo ardrone_autonomy changes)
	lastNavdataReceived.rotZ *= -1;	// yaw inverted
	lastNavdataReceived.rotY *= -1;	// pitch inverted
	lastNavdataReceived.vy *= -1;	// yaw inverted
	lastNavdataReceived.vz *= -1;	// pitch inverted
	lastNavdataReceived.ay *= -1;	// yaw inverted
	lastNavdataReceived.az *= -1;	// pitch inverted



	// push back in filter queue.
	pthread_mutex_lock( &filter->filter_CS );
	filter->navdataQueue->push_back(lastNavdataReceived);
	pthread_mutex_unlock( &filter->filter_CS );


	// give to PTAM (for scale estimation)
	ptamWrapper->newNavdata(&lastNavdataReceived);


	// save last timestamp
	if(lastNavStamp != ros::Time(0) && (lastNavdataReceived.header.stamp - lastNavStamp > ros::Duration(0.1)))
		std::cout << (lastNavdataReceived.header.stamp - lastNavStamp).toSec() << "s between two consecutive navinfos. This system requires Navinfo at 200Hz. If this error persists, set drone to debug mode and change publish freq in ardrone_autonomy" << std::endl;
	lastNavStamp = lastNavdataReceived.header.stamp;


	if(logfileIMU != NULL)
	{
		int pingNav = 0, pingVid = 0;
		pthread_mutex_lock(&logIMU_CS);
		if(logfileIMU != NULL)
			(*logfileIMU) << getMS(lastNavdataReceived.header.stamp) << " " << lastNavdataReceived.tm << " " <<
			lastNavdataReceived.vx << " " << lastNavdataReceived.vy << " " << lastNavdataReceived.altd << " " << lastNavdataReceived.rotX/1000.0 << " " << lastNavdataReceived.rotY/1000.0 << " " << lastNavdataReceived.rotZ/1000.0 << " " <<
			lastNavdataReceived.pressure << " " <<  0 << " " <<  0 << " " << 0 << " " <<	// control: roll pitch gaz yaw.
			pingNav << " " << pingVid << "\n";
		pthread_mutex_unlock(&logIMU_CS);
	}

}

void EstimationNode::velCb(const geometry_msgs::TwistConstPtr velPtr)
{
	geometry_msgs::TwistStamped ts;
	ts.header.stamp = ros::Time::now();
	ts.twist = *velPtr;

	// for some reason this needs to be inverted.
	// linear.y corresponds to ROLL
	ts.twist.linear.y *= -1;
	ts.twist.linear.x *= -1;
	ts.twist.angular.z *= -1;

	pthread_mutex_lock( &filter->filter_CS );
	filter->velQueue->push_back(ts);
	pthread_mutex_unlock( &filter->filter_CS );
}

void EstimationNode::vidCb(const sensor_msgs::ImageConstPtr img)
{
	// give to PTAM
	ptamWrapper->newImage(img);
}

void EstimationNode::comCb(const std_msgs::StringConstPtr str)
{
	if(str->data.length() > 2 && str->data.substr(0,2) == "p ")
	{
		ptamWrapper->handleCommand(str->data.substr(2,str->data.length()-2));
	}

	if(str->data.length() > 2 && str->data.substr(0,2) == "f ")
	{
		mapView->handleCommand(str->data.substr(2,str->data.length()-2));
	}

	if(str->data.length() > 2 && str->data.substr(0,2) == "m ")
	{
		mapView->handleCommand(str->data.substr(2,str->data.length()-2));
	}

	if(str->data.length() == 9 && str->data.substr(0,9) == "toggleLog")
	{
		this->toogleLogging();
	}





	int a, b;
	if(sscanf(str->data.c_str(),"pings %d %d",&a, &b) == 2)
	{
		filter->setPing((unsigned int)a, (unsigned int)b);
		predTime = ros::Duration((0.001*filter->delayControl));	// set predTime to new delayControl
	}
}


void EstimationNode::Loop()
{
	  ros::Rate pub_rate(publishFreq);

	  ros::Time lastInfoSent = ros::Time::now();

	  while (nh_.ok())
	  {
		  // -------------- 1. put nav & control in internal queues. ---------------
		  ros::spinOnce();


		  // -------------- 3. get predicted pose and publish! ---------------
		  // get filter state msg
		  pthread_mutex_lock( &filter->filter_CS );
		  tum_ardrone::filter_state s = filter->getPoseAt(ros::Time().now() + predTime);
		  pthread_mutex_unlock( &filter->filter_CS );

		  // fill metadata
		  s.header.stamp = ros::Time().now();
		  s.scale = filter->getCurrentScales()[0];
		  s.scaleAccuracy = filter->getScaleAccuracy();
		  s.ptamState = ptamWrapper->PTAMStatus;
		  s.droneState = lastNavdataReceived.state;
		  s.batteryPercent = lastNavdataReceived.batteryPercent;

		  // publish!
		  dronepose_pub.publish(s);


		  // --------- if need be: add fake PTAM obs --------
		  // if PTAM updates hang (no video or e.g. init), filter is never permanently rolled forward -> queues get too big.
		  // dont allow this to happen by faking a ptam observation if queue gets too big (500ms = 100 observations)
		  if((getMS(ros::Time().now()) - filter->predictdUpToTimestamp) > 500)
			  filter->addFakePTAMObservation(getMS(ros::Time().now()) - 300);


		  // ---------- maybe send new info --------------------------
		  if((ros::Time::now() - lastInfoSent) > ros::Duration(0.4))
		  {
			  reSendInfo();
			  lastInfoSent = ros::Time::now();
		  }

		  // -------------- 4. sleep until rate is hit. ---------------
		  pub_rate.sleep();
	  }
}
void EstimationNode::dynConfCb(tum_ardrone::StateestimationParamsConfig &config, uint32_t level)
{
	if(!filter->allSyncLocked && config.PTAMSyncLock)
		ROS_WARN("Ptam Sync has been disabled. This fixes scale etc.");

	if(!ptamWrapper->mapLocked && config.PTAMMapLock)
		ROS_WARN("Ptam Map has been locked.");


	filter->useControl =config.UseControlGains;
	filter->usePTAM =config.UsePTAM;
	filter->useNavdata =config.UseNavdata;

	filter->useScalingFixpoint = config.RescaleFixOrigin;

	ptamWrapper->maxKF = config.PTAMMaxKF;
	ptamWrapper->mapLocked = config.PTAMMapLock;
	filter->allSyncLocked = config.PTAMSyncLock;


	ptamWrapper->setPTAMPars(config.PTAMMinKFTimeDiff, config.PTAMMinKFWiggleDist, config.PTAMMinKFDist);


	filter->c1 = config.c1;
	filter->c2 = config.c2;
	filter->c3 = config.c3;
	filter->c4 = config.c4;
	filter->c5 = config.c5;
	filter->c6 = config.c6;
	filter->c7 = config.c7;
	filter->c8 = config.c8;

}

pthread_mutex_t EstimationNode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER; //pthread_mutex_lock( &cs_mutex );
void EstimationNode::publishCommand(std::string c)
{
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&tum_ardrone_CS);
	tum_ardrone_pub.publish(s);
	pthread_mutex_unlock(&tum_ardrone_CS);
}

void EstimationNode::publishTf(TooN::SE3<> trans, ros::Time stamp, int seq, std::string system)
{
	trans = trans.inverse();

	tf::Matrix3x3 m;
	m[0][0] = trans.get_rotation().get_matrix()(0,0);
	m[0][1] = trans.get_rotation().get_matrix()(0,1);
	m[0][2] = trans.get_rotation().get_matrix()(0,2);
	m[1][0] = trans.get_rotation().get_matrix()(1,0);
	m[1][1] = trans.get_rotation().get_matrix()(1,1);
	m[1][2] = trans.get_rotation().get_matrix()(1,2);
	m[2][0] = trans.get_rotation().get_matrix()(2,0);
	m[2][1] = trans.get_rotation().get_matrix()(2,1);
	m[2][2] = trans.get_rotation().get_matrix()(2,2);

	tf::Vector3 v;
	v[0] = trans.get_translation()[0];
	v[1] = trans.get_translation()[1];
	v[2] = trans.get_translation()[2];


	tf::Transform tr = tf::Transform(m,v);
	tf::StampedTransform t = tf::StampedTransform(tr,stamp,"map",system);
	tf_broadcaster.sendTransform(t);



	if(logfilePTAMRaw != NULL)
	{
		pthread_mutex_lock(&(logPTAMRaw_CS));
		// log:
		// - filterPosePrePTAM estimated for videoFrameTimestamp-delayVideo.
		// - PTAMResulttransformed estimated for videoFrameTimestamp-delayVideo. (using imu only for last step)
		// - predictedPoseSpeed estimated for lastNfoTimestamp+filter->delayControl	(actually predicting)
		// - predictedPoseSpeedATLASTNFO estimated for lastNfoTimestamp	(using imu only)
		if(logfilePTAMRaw != NULL)
			(*(logfilePTAMRaw)) << seq << " " << stamp << " " << tr.getOrigin().x() << " " << tr.getOrigin().y() << " " << tr.getOrigin().z() << " " <<
			tr.getRotation().x() << " " << tr.getRotation().y() << " " << tr.getRotation().z() << " " << tr.getRotation().w() << std::endl;

		pthread_mutex_unlock(&(logPTAMRaw_CS));
	}

}

void EstimationNode::toogleLogging()
{
	// first: always check for /log dir
	struct stat st;
	if(stat((packagePath+std::string("/logs")).c_str(),&st) != 0)
		mkdir((packagePath+std::string("/logs")).c_str(),S_IXGRP | S_IXOTH | S_IXUSR | S_IRWXU | S_IRWXG | S_IROTH);

	char buf[200];
	bool quitLogging = false;
	if(logfileIMU == 0)
	{
		currentLogID = ((long)time(0))*100+(getMS()%100);		// time(0) + ms
		startedLogClock = getMS();
		ROS_INFO("\n\nENABLED LOGGING to %s/logs/%ld\n\n\n",packagePath.c_str(),currentLogID);
		sprintf(buf,"%s/logs/%ld",packagePath.c_str(),currentLogID);
		mkdir(buf, S_IXGRP | S_IXOTH | S_IXUSR | S_IRWXU | S_IRWXG | S_IROTH);


		sprintf(buf,"u l ENABLED LOGGING to %s/logs/%ld",packagePath.c_str(),currentLogID);
		publishCommand(buf);
	}
	else
		quitLogging = true;



	// IMU
	pthread_mutex_lock(&logIMU_CS);
	if(logfileIMU == 0)
	{
		logfileIMU = new std::ofstream();
		sprintf(buf,"%s/logs/%ld/logIMU.txt",packagePath.c_str(),currentLogID);
		logfileIMU->open (buf);
	}
	else
	{
		logfileIMU->flush();
		logfileIMU->close();
		delete logfileIMU;
		logfileIMU = NULL;
	}
	pthread_mutex_unlock(&logIMU_CS);


	// IMU
	pthread_mutex_lock(&logPTAM_CS);
	if(logfilePTAM == 0)
	{
		logfilePTAM = new std::ofstream();
		sprintf(buf,"%s/logs/%ld/logPTAM.txt",packagePath.c_str(),currentLogID);
		logfilePTAM->open (buf);
	}
	else
	{
		logfilePTAM->flush();
		logfilePTAM->close();
		delete logfilePTAM;
		logfilePTAM = NULL;
	}
	pthread_mutex_unlock(&logPTAM_CS);



	// IMU
	pthread_mutex_lock(&logFilter_CS);
	if(logfileFilter == 0)
	{
		logfileFilter = new std::ofstream();
		sprintf(buf,"%s/logs/%ld/logFilter.txt",packagePath.c_str(),currentLogID);
		logfileFilter->open (buf);
	}
	else
	{
		logfileFilter->flush();
		logfileFilter->close();
		delete logfileFilter;
		logfileFilter = NULL;
	}
	pthread_mutex_unlock(&logFilter_CS);


	// IMU
	pthread_mutex_lock(&logPTAMRaw_CS);
	if(logfilePTAMRaw == 0)
	{
		logfilePTAMRaw = new std::ofstream();
		sprintf(buf,"%s/logs/%ld/logPTAMRaw.txt",packagePath.c_str(),currentLogID);
		logfilePTAMRaw->open (buf);
	}
	else
	{
		logfilePTAMRaw->flush();
		logfilePTAMRaw->close();
		delete logfilePTAMRaw;
		logfilePTAMRaw = NULL;
	}
	pthread_mutex_unlock(&logPTAMRaw_CS);


	if(quitLogging)
	{
		printf("\n\nDISABLED LOGGING (logged %ld sec)\n\n\n",(getMS()-startedLogClock+500)/1000);
		char buf2[200];
		sprintf(buf,"%s/logs/%ld",packagePath.c_str(),currentLogID);
		sprintf(buf2,"%s/logs/%ld-%lds",packagePath.c_str(),currentLogID,(getMS()-startedLogClock+500)/1000);
		rename(buf,buf2);
	}

}

void EstimationNode::reSendInfo()
{

	// get ptam status string
	std::string ptamStatus;
	switch(ptamWrapper->PTAMStatus)
	{
	case PTAMWrapper::PTAM_IDLE:
		ptamStatus = "Idle";
		break;
	case PTAMWrapper::PTAM_INITIALIZING:
		ptamStatus = "Initializing";
		break;
	case PTAMWrapper::PTAM_LOST:
		ptamStatus = "Lost";
		break;
	case PTAMWrapper::PTAM_FALSEPOSITIVE:
		ptamStatus = "FalsePositive";
		break;
	case PTAMWrapper::PTAM_GOOD:
		ptamStatus = "Good";
		break;
	case PTAMWrapper::PTAM_TOOKKF:
	case PTAMWrapper::PTAM_BEST:
		ptamStatus = "Best";
		break;
	}



	// parse PTAM message
	std::string ptamMsg = ptamWrapper->lastPTAMMessage;
	int kf, kp, kps[4], kpf[4];
	int pos = ptamMsg.find("Found: ");
	int found = 0;
	if(pos != std::string::npos)
		found = sscanf(ptamMsg.substr(pos).c_str(),"Found: %d/%d %d/%d %d/%d %d/%d Map: %dP, %dKF",
						&kpf[0],&kps[0],&kpf[1],&kps[1],&kpf[2],&kps[2],&kpf[3],&kps[3],&kp,&kf);
	char bufp[200];
	if(found == 10)
		snprintf(bufp,200,"Map: KF: %d, KP: %d (%d of %d found)",
				kf, kp,kpf[0]+kpf[1]+kpf[2]+kpf[3], kps[0]+kps[1]+kps[2]+kps[3]);
	else
		snprintf(bufp,200,"Map: -");

	lastNavdataReceived.batteryPercent;

	std::string status = "";
	switch(	lastNavdataReceived.state)
	{
		case 0: status = "Unknown";break;
		case 1: status = "Init"; break;
		case 2: status = "Landed";break;
		case 3: status = "Flying"; break;
		case 4: status = "Hovering";break;
		case 5: status = "Test"; break;
		case 6: status = "Taking off";break;
		case 7: status = "Goto Fix Point"; break;
		case 8: status = "Landing";break;
		case 9: status = "Looping"; break;
	}
	/*
	PTAM: Idle | Good | Dodgy | Lost
	Map: KF: X, KP: X (X searched, X found)
	Scale: X (in: X, out: x), acc: X
	Scale Fixpoint: NONE | DRONE
	Status: X (Battery: X)
	*/
	char buf[1000];
	snprintf(buf,1000,"u s PTAM: %s\n%s\nScale: %.3f (%d in, %d out), acc: %.2f\nScaleFixpoint: %s\nDrone Status: %s (%d Battery)",
			ptamStatus.c_str(),
			bufp,
			filter->getCurrentScales()[0],filter->scalePairsIn,filter->scalePairsOut,filter->getScaleAccuracy(),
			filter->useScalingFixpoint ? "FIX" : "DRONE",
			status.c_str(), (int)lastNavdataReceived.batteryPercent);

	publishCommand(buf);
}
