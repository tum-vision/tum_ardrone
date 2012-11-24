#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 

#include "KIProcedure.h"

class KIFlyTo : public KIProcedure
{
private:
	int reachedAtClock;
	bool reached;
	bool targetSet;
	bool isCompleted;

	int stayTimeMs;
	double maxControlFactor;
	double initialReachedDist;
	double stayWithinDist;


	DronePosition checkpoint;

public:
	KIFlyTo(DronePosition checkpoint, 
		double stayTime = 2,
		double maxControlFactorP = 1,
		double initialReachedDistP = 0.2,
		double stayWithinDistP = 0.5);

	~KIFlyTo(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

