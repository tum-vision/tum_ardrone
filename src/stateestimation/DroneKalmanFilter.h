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
#ifndef __DRONEKALMANFILTER_H
#define __DRONEKALMANFILTER_H

#include <TooN/TooN.h>
#include <deque>
#include <iostream>
#include <fstream>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <pthread.h>
#include "../HelperFunctions.h"
#include "tum_ardrone/filter_state.h"

class EstimationNode;


class ScaleStruct
{
public:
	TooN::Vector<3> ptam;
	TooN::Vector<3> imu;
	double ptamNorm;
	double imuNorm;
	double alphaSingleEstimate;
	double pp, ii, pi;

	inline double computeEstimator(double spp, double sii, double spi, double stdDevPTAM = 0.2, double stdDevIMU = 0.1)
	{
		double sII = stdDevPTAM * stdDevPTAM * sii;
		double sPP = stdDevIMU * stdDevIMU * spp;
		double sPI = stdDevIMU * stdDevPTAM * spi;

		double tmp = (sII-sPP)*(sII-sPP) + 4*sPI*sPI;
		if(tmp <= 0) tmp = 1e-5;	// numeric issues
		return 0.5*((sII-sPP)+sqrt(tmp)) / (stdDevPTAM * stdDevPTAM * spi);

	}

	inline ScaleStruct(TooN::Vector<3> ptamDist, TooN::Vector<3> imuDist)
	{
		ptam = ptamDist;
		imu = imuDist;
		pp = ptam[0]*ptam[0] + ptam[1]*ptam[1] + ptam[2]*ptam[2];
		ii = imu[0]*imu[0] + imu[1]*imu[1] + imu[2]*imu[2];
		pi = imu[0]*ptam[0] + imu[1]*ptam[1] + imu[2]*ptam[2];

		ptamNorm = sqrt(pp);
		imuNorm = sqrt(ii);

		alphaSingleEstimate = computeEstimator(pp,ii,pi);
	}

	inline bool operator < (const ScaleStruct& comp) const
	{
		return alphaSingleEstimate < comp.alphaSingleEstimate;
	}
};


// KalmanFilter with two components (pose, speed)
class PVFilter
{
public:
	TooN::Vector<2> state;
	TooN::Matrix<2,2> var;
	

	// constructors
	inline PVFilter(TooN::Vector<2> state, TooN::Matrix<2,2> var)
		: state(state), var(var)
	{
	}

	inline PVFilter(double pose, double speed)
		: state(TooN::makeVector(pose,speed)), var(TooN::Zeros)
	{
	}

	inline PVFilter(double pose)
		: state(TooN::makeVector(pose,0)), var(TooN::Zeros)
	{
		var(1,1) = 1e10;
	}

	inline PVFilter()
		: state(TooN::makeVector(0,0)), var(TooN::Zeros)
	{
		var(1,1) = var(0,0) = 1e10;
	}

	// observe
	inline void observePose(double obs, double obsVar)
	{
		/* MATLAB:
		H = [1 0];
		K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
		state = state + K * (obs - H*state);
		var = (eye(2)-K*H) * var;
		*/
		TooN::Vector<2> K = var[0] / (obsVar + var(0,0));	//K is first col = first row of var.
		state = state + K * (obs - state[0]);
		TooN::Matrix<2> tmp = TooN::Identity;
		tmp(0,0) -= K[0]; 
		tmp(1,0) -= K[1];
		var = tmp * var;

	}


	inline void observeSpeed(double obs, double obsVar)
	{
		/* MATLAB:
		H = [0 1];
		K = (uncertainty * H') / ((H * uncertainty * H') + obsVar);
		state = state + K * (observation - H*state);
		uncertainty = (eye(2)-K*H) * uncertainty;
		*/
		TooN::Vector<2> K = var[1] / (obsVar + var(1,1));	//K is second col = second row of var.
		state = state + K * (obs - state[1]);
		TooN::Matrix<2> tmp = TooN::Identity;
		tmp(0,1) -= K[0]; 
		tmp(1,1) -= K[1];
		var = tmp * var;
	}


	// predict
	// calculates prediction variance matrix based on gaussian acceleration as error.
	inline void predict(double ms, double accelerationVar, TooN::Vector<2> controlGains = TooN::makeVector(0,0), double coVarFac = 1, double speedVarFac = 1)
	{
		/* MATLAB:
		G = [1 ms/1000; 0 1];
		E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
		state = G*state;
		var = G * var * G' + accelerationVarPerS*(E*E');
		*/

		ms /= 1000;

		TooN::Matrix<2,2> G = TooN::Identity;
		G(0,1) = ms;

		state = G * state + controlGains;
		var  = G * var * G.T();
		var(0,0) += accelerationVar * 0.25 * ms*ms*ms*ms;
		var(1,0) += coVarFac * accelerationVar * 0.5 * ms*ms*ms * 4;
		var(0,1) += coVarFac * accelerationVar * 0.5 * ms*ms*ms * 4;
		var(1,1) += speedVarFac * accelerationVar * 1 * ms*ms * 4 * 4;
	}

	// predict
	// calculates prediction using the given uncertainty matrix
	// vars is var(0) var(1) covar(0,1)
	inline void predict(double ms, TooN::Vector<3> vars, TooN::Vector<2> controlGains = TooN::makeVector(0,0))
	{
		/* MATLAB:
		G = [1 ms/1000; 0 1];
		E = [((ms/1000)^2)/2; ms/1000]; % assume normal distributed constant ACCELERATION.
		state = G*state;
		var = G * var * G' + accelerationVarPerS*(E*E');
		*/

		ms /= 1000;

		TooN::Matrix<2,2> G = TooN::Identity;
		G(0,1) = ms;

		state = G * state + controlGains;
		var  = G * var * G.T();
		var(0,0) += vars[0];
		var(1,0) += vars[2];
		var(0,1) += vars[2];
		var(1,1) += vars[1];
	}
};


// KalmanFilter with only one component (pose, is observed directly)
class PFilter
{
public:
	double state;
	double var;
	
	inline PFilter() : state(0), var(1e10) {};
	inline PFilter(double initState) : state(initState), var(0) {};

	inline void predict(double ms, double speedVar, double controlGains = 0)
	{
		/* MATLAB:
		state = state;
		var = var + speedVar*((ms/1000)^2);
		*/
		state += controlGains;
		var += speedVar * ms * ms / 1000000;
	}

	inline void observe(double obs, double obsVar)
	{
		/* MATLAB:
		obs_w = var / (var + obsVar);
		state = state * (1-obs_w) + obs * obs_w;
		var = var*obsVar / (var + obsVar);
		*/
		double w = var / (var + obsVar);
		state = (1-w) * state + w * obs;
		var = var * obsVar / (var + obsVar);
	}
};


class DroneKalmanFilter
{
private:
	// filter
	PVFilter x;
	PVFilter y;
	PVFilter z;
	PFilter roll;
	PFilter pitch;
	PVFilter yaw;


	// relation parameters (ptam to imu scale / offset)
	double x_offset, y_offset, z_offset;
	double xy_scale, z_scale;
	double scale_from_z;
	double scale_from_xy;
	double roll_offset, pitch_offset, yaw_offset;
	bool offsets_xyz_initialized;
	bool scale_xyz_initialized;

	// intermediate values for re-estimation of relaton parameters
	double xyz_sum_IMUxIMU;
	double xyz_sum_PTAMxPTAM;
	double xyz_sum_PTAMxIMU;
	double rp_offset_framesContributed;
	std::vector<ScaleStruct>* scalePairs;

	// parameters used for height and yaw differentiation
	double last_yaw_IMU;
	double last_z_IMU;
	long last_yaw_droneTime;
	long last_z_droneTime;
	long last_z_packageID;
	
	// contains the last ptam-pose added (raw ptam-data).
	TooN::Vector<3> last_PTAM_pose;

	// contains the pose directly after the last ptam-fuse.
	TooN::Vector<3> last_fused_pose;
	bool lastPosesValid;


	// statistics parameters
	int numGoodIMUObservations;
	int numGoodPTAMObservations;

	// parameters used for adding / timing
	long lastIMU_XYZ_dronetime;
	long lastIMU_RPY_dronetime;
	long lastIMU_dronetime;
	int lastIMU_XYZ_ID;
	int lastIMU_RPY_ID;
	double lastPredictedRoll;
	double lastPredictedPitch;
	double initialScaleSet;

	// internal add functions
	void predictInternal(geometry_msgs::Twist activeControlInfo, int timeSpanMicros, bool useControlGains = true);
	void observeIMU_XYZ(const ardrone_autonomy::Navdata* nav);
	void observeIMU_RPY(const ardrone_autonomy::Navdata* nav);
	void observePTAM(TooN::Vector<6> pose);


	// internal sync functions. called on ptam-add.
	void sync_xyz(double x_global, double y_global, double z_global);
	void sync_rpy(double roll_global, double pitch_global, double yaw_global);

	// ms state of filter
	int predictedUpToTotal;
	long predictedUpToDroneTime;

	// logging
	double lastdYaw, lastdZ;
	double baselineZ_IMU;
	double baselineZ_Filter;
	double last_z_heightDiff;
	double baselineY_IMU;
	double baselineY_Filter;
	bool baselinesYValid;
	int timestampYawBaselineFrom;
	double lastVXGain;
	double lastVYGain;



	EstimationNode* node;
public:
	DroneKalmanFilter(EstimationNode* node);
	~DroneKalmanFilter(void);
	void release();

	static int delayRPY;	// assumed 0 (fastest data).
	static int delayXYZ;	// assumed 70 (gets here 70ms later than rpy)
	static int delayVideo;	// assumed 120 (gets here 120ms later than rpy)
	static int delayControl;	// assumed 120 (gets here 120ms later than rpy)
	
	static const int base_delayXYZ;
	static const int base_delayVideo;
	static const int base_delayControl;

	std::deque<ardrone_autonomy::Navdata>* navdataQueue;	// new navdata messages
	std::deque<geometry_msgs::TwistStamped>* velQueue;		// new velocity messages
	static pthread_mutex_t filter_CS;


	int predictdUpToTimestamp;
	int scalePairsIn, scalePairsOut;



	// resets everything to zero.
	void reset();


	// resets everything to do with PTAM to zero (call if tracking continues, but PTAM tracking is reset)
	void clearPTAM();

	// predicts up to a specified time in ms, using all available data.
	// if consume=false, does not delete anything from queues.
	void predictUpTo(int timestamp, bool consume = true, bool useControlGains = true);
	
	void setPing(unsigned int navPing, unsigned int vidPing);

	// gets current pose and variances (up to where predictUpTo has been called)
	TooN::Vector<6> getCurrentPose();
	tum_ardrone::filter_state getCurrentPoseSpeed();
	TooN::Vector<10> getCurrentPoseSpeedAsVec();
	TooN::Vector<10> getCurrentPoseSpeedVariances();
	TooN::Vector<6> getCurrentPoseVariances();
	TooN::Vector<6> getCurrentOffsets();
	TooN::Vector<3> getCurrentScales();
	TooN::Vector<3> getCurrentScalesForLog();
	float getScaleAccuracy();
	void setCurrentScales(TooN::Vector<3> scales);

	// adds a PTAM observation. automatically predicts up to timestamp.
	void updateScaleXYZ(TooN::Vector<3> ptamDiff, TooN::Vector<3> imuDiff, TooN::Vector<3> OrgPtamPose);


	// does not actually change the state of the filter.
	// makes a compy of it, flushes all queued navdata into it, then predicts up to timestamp.
	// relatively costly (!)

	// transforms a PTAM observation.
	// translates from front to center, scales and adds offsets.
	TooN::Vector<3> transformPTAMObservation(double x,double y,double z);
	TooN::Vector<3> transformPTAMObservation(double x,double y,double z, double yaw);
	TooN::Vector<6> transformPTAMObservation(TooN::Vector<6> obs);
	TooN::Vector<6> backTransformPTAMObservation(TooN::Vector<6> obs);

	inline int getNumGoodPTAMObservations() {return numGoodPTAMObservations;}

	// when scaling factors are updated, exacly one point stays the same.
	// if useScalingFixpoint, this point is the current PTAM pose, otherwise it is sclingFixpoint (which is a PTAM-coordinate(!))
	TooN::Vector<3> scalingFixpoint;	// in PTAM's system (!)
	bool useScalingFixpoint;

	//
	void flushScalePairs();
	
	// locking
	bool allSyncLocked;
	bool useControl;
	bool useNavdata;
	bool usePTAM;

	// motion model parameters
	float c1;
	float c2;
	float c3;
	float c4;
	float c5;
	float c6;
	float c7;
	float c8;



	bool handleCommand(std::string s);

	// new ROS interface functions
	void addPTAMObservation(TooN::Vector<6> trans, int time);
	void addFakePTAMObservation(int time);
	tum_ardrone::filter_state getPoseAt(ros::Time t, bool useControlGains = true);
	TooN::Vector<10> getPoseAtAsVec(int timestamp, bool useControlGains = true);

};
#endif /* __DRONEKALMANFILTER_H */
