/*
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * 
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
 
 
#include "Predictor.h"
#include "../HelperFunctions.h"

const TooN::SE3<double> Predictor::droneToBottom = TooN::SE3<double>(TooN::SO3<double>(TooN::makeVector(3.14159265,0,0)),TooN::makeVector(0,0,0));
const TooN::SE3<double> Predictor::bottomToDrone = Predictor::droneToBottom.inverse();

const TooN::SE3<double> Predictor::droneToFront = TooN::SE3<double>(TooN::SO3<double>(TooN::makeVector(3.14159265/2,0,0)),TooN::makeVector(0,0.025,-0.2));
const TooN::SE3<double> Predictor::frontToDrone = Predictor::droneToFront.inverse();

const TooN::SE3<double> Predictor::droneToFrontNT = TooN::SE3<double>(TooN::SO3<double>(TooN::makeVector(3.14159265/2,0,0)),TooN::makeVector(0,0,0));
const TooN::SE3<double> Predictor::frontToDroneNT = Predictor::droneToFrontNT.inverse();


// load distortion matrices
Predictor::Predictor(std::string basePath)
{

	setPosRPY(0,0,0,0,0,0);
	lastAddedDronetime = 0;
}


Predictor::~Predictor(void)
{
}

void Predictor::calcCombinedTransformations()
{
	globalToFront = droneToFront * globaltoDrone;
	globalToBottom = droneToBottom * globaltoDrone;
	frontToGlobal = globalToFront.inverse();
	bottmoToGlobal = globalToBottom.inverse();
}


// input in rpy
void Predictor::setPosRPY(double newX, double newY, double newZ, double newRoll, double newPitch, double newYaw)
{
	// set rpy
	x = newX; y = newY; z = newZ;
	roll = newRoll; pitch = newPitch; yaw = newYaw;

	// set se3
	droneToGlobal.get_translation()[0] = x;
	droneToGlobal.get_translation()[1] = y;
	droneToGlobal.get_translation()[2] = z;
	droneToGlobal.get_rotation() = rpy2rod(roll,pitch,yaw);

	globaltoDrone = droneToGlobal.inverse();

	// set rest
	calcCombinedTransformations();
}

// input in SE3
void Predictor::setPosSE3_globalToDrone(TooN::SE3<double> newGlobaltoDrone)
{
	// set se3
	globaltoDrone = newGlobaltoDrone;
	droneToGlobal = globaltoDrone.inverse();

	// set rpy
	x = droneToGlobal.get_translation()[0];
	y = droneToGlobal.get_translation()[1];
	z = droneToGlobal.get_translation()[2];
	rod2rpy(droneToGlobal.get_rotation(),&roll,&pitch,&yaw);

	// set rest
	calcCombinedTransformations();
}
void Predictor::setPosSE3_droneToGlobal(TooN::SE3<double> newDroneToGlobal)
{
	droneToGlobal = newDroneToGlobal;
	globaltoDrone = droneToGlobal.inverse();
	
	x = droneToGlobal.get_translation()[0];
	y = droneToGlobal.get_translation()[1];
	z = droneToGlobal.get_translation()[2];

	rod2rpy(droneToGlobal.get_rotation(),&roll,&pitch,&yaw);

	calcCombinedTransformations();
}


// watch out: does NOT update any matrices, only (x,y,z,r,p,y)!!!!!!!
// also: does not filter z-data, only sets corrupted-flag...
void Predictor::predictOneStep(ardrone_autonomy::Navdata* nfo)
{
	double timespan = nfo->tm - lastAddedDronetime;	// in micros
	lastAddedDronetime = nfo->tm;
	if(timespan > 50000 || timespan < 1)
		timespan = std::max(0.0,std::min(5000.0,timespan));	// clamp strange values

	// horizontal speed integration
	// (mm / s)/1.000 * (mics/1.000.000) = meters.
	double dxDrone = nfo->vx * timespan / 1000000000;	// in meters
	double dyDrone = nfo->vy * timespan / 1000000000;	// in meters

	double yawRad = (nfo->rotZ/1000.0) / (180.0/3.1415);
	x += sin(yawRad)*dxDrone+cos(yawRad)*dyDrone;
	y += cos(yawRad)*dxDrone-sin(yawRad)*dyDrone;

	// height
	if(abs(z - (double)nfo->altd*0.001) > 0.12)
	{
		if(std::abs(z - (double)nfo->altd*0.001) > abs(zCorruptedJump))
			zCorruptedJump = z - (double)nfo->altd*0.001;
		zCorrupted = true;
	}

	z = nfo->altd*0.001;

	// angles
	roll = nfo->rotX/1000.0;
	pitch = nfo->rotY/1000.0;
	yaw = nfo->rotZ/1000.0;

}
void Predictor::resetPos()
{
	zCorrupted = false;
	zCorruptedJump = 0;
	setPosRPY(0,0,0,0,0,0);
}
