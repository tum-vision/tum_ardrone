/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
#include "KIFlyTo.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIFlyTo::KIFlyTo(DronePosition checkpointP, 
		double stayTime,
		double maxControlFactorP,
		double initialReachedDistP,
		double stayWithinDistP)
{
	stayTimeMs = (int)(1000*stayTime);
	maxControlFactor = maxControlFactorP;
	initialReachedDist = initialReachedDistP;
	stayWithinDist = stayWithinDistP;

	checkpoint = checkpointP;

	reachedAtClock = -1;
	reached = false;
	
	targetSet = false;

	isCompleted = false;

	char buf[200];
	snprintf(buf,200,"goto %.2f %.2f %.2f %.2f", checkpointP.pos[0], checkpointP.pos[1], checkpointP.pos[2], checkpointP.yaw);
	command = buf;
}


KIFlyTo::~KIFlyTo(void)
{
}


bool KIFlyTo::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(!targetSet)
		controller->setTarget(checkpoint);
	targetSet = true;

	// target reached?
	if(!isCompleted && reached && (getMS() - reachedAtClock) > stayTimeMs)
	{
		printf("checkpoint done!\n");
		isCompleted = true;
	}
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}


	// get target dist:
	TooN::Vector<3> diffs = TooN::makeVector(
			statePtr->x - checkpoint.pos[0],
			statePtr->y - checkpoint.pos[1],
			statePtr->z - checkpoint.pos[2]);

	double diffYaw = statePtr->yaw - checkpoint.yaw;
	double diffDistSquared = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

	// if not reached yet, need to get within small radius to count.
	if(!reached && diffDistSquared < initialReachedDist * initialReachedDist && diffYaw < 5)
	{
		reached = true;
		reachedAtClock = getMS();
		printf("target reached initially!\n");
	}

	// if too far away again: revoke reached status...
	if(reached && (diffDistSquared > stayWithinDist * stayWithinDist || diffYaw > 5))
	{
		reached = false;
		printf("target lost again!\n");
	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
