#pragma once
/*
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * 
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
 
#include "TooN/TooN.h"
#include "TooN/so3.h"
#include "TooN/se3.h"
#include <string>
#include <ardrone_autonomy/Navdata.h>


// handles the drone's coordinate frames.
// drone: coordinate system of drone. at zero equals global CS
//		  positive z: "up"
//		  positive x: "right"
//		  positive y: "front"

// front: axis directions as in drone.
//		  center at (0,0.2,0.025)_drone
//		  positive z: "front"
//		  positive x: "right"
//		  positive y: "down"

// bottom: 
//		  positive z: "down"
//		  positive x: "right"
//		  positive y: "back"
//		  center at (0,0,0)_drone

// scale is meters.
class Predictor
{
private:
	void calcGtDRodTransFromSE3();
	void calcDtGRodTransFromSE3();
	void calcRPYXYZFromRodDisp();
	void calcCombinedTransformations();

public:

	// --------------------- static transformation matrices ------------------------
	// matrix from bottom cam CO to drone CO
	static const TooN::SE3<double> bottomToDrone;
	static const TooN::SE3<double> droneToBottom;

	// matrix from front cam CO to drone CO
	static const TooN::SE3<double> frontToDrone;
	static const TooN::SE3<double> droneToFront;

	// matrix from front cam CO to drone CO, without translation (!)
	static const TooN::SE3<double> frontToDroneNT;
	static const TooN::SE3<double> droneToFrontNT;

	// --------------------- current drone state in various represenatations -----------------------
	// current quadcopter position saved in three ways:
	// as SE3 transformation (matrix+displacement)
	TooN::SE3<double> globaltoDrone;	//translation is globalToDroneDisp; rotation is matrix of globalToDroneRod
	TooN::SE3<double> droneToGlobal;	//translation is droneToGlobalDisp=(x,y,z); rotation is matrix of droneToGlobalRod

	TooN::SE3<double> globalToFront;
	TooN::SE3<double> frontToGlobal;
	TooN::SE3<double> globalToBottom;
	TooN::SE3<double> bottmoToGlobal;


	// xyz-position is center of drone CS in global coordinates.
	// rpy-rotation is rpy of drone.
	double roll;
	double pitch;
	double yaw;
	double x;
	double y;
	double z;
	bool zCorrupted;
	double lastAddedDronetime;
	double zCorruptedJump;



	// ------------------------- set internal pose from some representation.-----------------------------------------
	// all representations are automatically adjusted.
	void setPosRPY(double newX, double newY, double newZ, double newRoll, double newPitch, double newYaw);
	void setPosSE3_globalToDrone(TooN::SE3<double> newGlobaltoDrone);
	void setPosSE3_droneToGlobal(TooN::SE3<double> newDroneToGlobal);

	// -------------------------- prediction -----------------------------------------------------------------------

	void predictOneStep(ardrone_autonomy::Navdata* nfo);
	void resetPos();
	
	Predictor(std::string basePath="");
	~Predictor(void);
};

