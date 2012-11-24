/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
#include "KILand.h"
#include "../DroneController.h"
#include "../ControlNode.h"


KILand::KILand(void)
{
	fresh = true;
	command = "land";
}


KILand::~KILand(void)
{
}

bool KILand::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(fresh)
	{
		node->sendLand();
		printf("issuing land!\n");
	}
	fresh = false;

	// TODO: maybe do something better here, like still controlling x, y, yaw pos...
	node->sendControlToDrone(node->hoverCommand);
	return true;
}
