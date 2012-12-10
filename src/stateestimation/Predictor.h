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
#ifndef __PREDICTOR_H
#define __PREDICTOR_H
 
 
 
 
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
#endif /* __PREDICTOR_H */

