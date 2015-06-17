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
	controller->clearTarget();
	return true;
}
