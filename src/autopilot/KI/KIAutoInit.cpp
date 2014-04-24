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

#include "KIAutoInit.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIAutoInit::KIAutoInit(bool resetMap, int imoveTimeMS, int iwaitTimeMS, int reachHeightMS, float controlMult, bool takeoff)
{
	stage = NONE;
	this->resetMap = resetMap;
	moveTimeMS = imoveTimeMS;
	waitTimeMS = iwaitTimeMS;
	this->reachHeightMS = reachHeightMS;
	this->controlCommandMultiplier = controlMult;

	nextUp = false;
	stageStarted = false;

	if(!takeoff)
		stage = WAIT_FOR_FIRST;

	char buf[200];
	if(resetMap)
		snprintf(buf,200,"autoInit %d %d", imoveTimeMS, iwaitTimeMS);
	else
		snprintf(buf,200,"takeoff");

	command = buf;
}


KIAutoInit::~KIAutoInit(void)
{
}


bool KIAutoInit::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	// no PTAM initialization, just takeoff.
	if(!resetMap)
	{
		switch(stage)
		{
		case NONE:		// start and proceed
			node->sendTakeoff();
			stageStarted = getMS();
			stage = WAIT_FOR_SECOND;
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case WAIT_FOR_SECOND:
			if(getMS() - stageStarted < 5000)
			{
				node->sendControlToDrone(node->hoverCommand);
				return false;
			}

			controller->setTarget(DronePosition(
					TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
			node->sendControlToDrone(controller->update(statePtr));
			stage = DONE;
			return true;
		case DONE:
			node->sendControlToDrone(controller->update(statePtr));
			return true;
		default:
			return false;
		}
		return true;	// should never happen....
	}
	else
	{
		switch(stage)
		{
		case NONE:		// start and proceed
			node->sendTakeoff();
			node->publishCommand("f reset");	// reset whole filter.

			stageStarted = getMS();
			stage = STARTED;
			nextUp = true;
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case STARTED:	// wait 6s to reach hight.
			if(getMS() - stageStarted > reachHeightMS)
			{
				stageStarted = getMS();
				stage = WAIT_FOR_FIRST;
			}
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case WAIT_FOR_FIRST:	// wait 1s and press space
			if(getMS() - stageStarted > 1000)
			{
				node->publishCommand("p space");
				stageStarted = getMS();
				stage = TOOK_FIRST;
			}
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case TOOK_FIRST:	// go [up/down] 1s and press space if was initializing.
			if(getMS() - stageStarted < moveTimeMS)
			{
				if(nextUp)
					node->sendControlToDrone(ControlCommand(0,0,0,0.6*controlCommandMultiplier));
				else
					node->sendControlToDrone(ControlCommand(0,0,0,-0.3*controlCommandMultiplier));
			}
			else if(getMS() - stageStarted < moveTimeMS+waitTimeMS)
			{
				node->sendControlToDrone(node->hoverCommand);
			}
			else	// time is up, take second KF
			{
				if(statePtr->ptamState == statePtr->PTAM_INITIALIZING)	// TODO: ptam status enum, this should be PTAM_INITIALIZING
				{
					node->publishCommand("p space");
					stageStarted = getMS();
					stage = WAIT_FOR_SECOND;
				}
				else	// sth was wrong: try again
				{
					nextUp = !nextUp;
					node->publishCommand("p reset");
					stageStarted = getMS();
					stage = WAIT_FOR_FIRST;
				}
				node->sendControlToDrone(node->hoverCommand);
			}
			return false;

		case WAIT_FOR_SECOND:

			// am i done?
			if(statePtr->ptamState == statePtr->PTAM_BEST || statePtr->ptamState == statePtr->PTAM_GOOD || statePtr->ptamState == statePtr->PTAM_TOOKKF) // TODO: PTAM_GOOD or PTAM_BEST or PTAM_TOOKKF
			{
				controller->setTarget(DronePosition(
									TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
				node->sendControlToDrone(controller->update(statePtr));
				stage = DONE;
				return true;
			}

			// am i stil waiting?
			// TODO: change this to something that becomes false, as soon as fail is evident.
			if(getMS() - stageStarted < 2500)	// wait 2s
			{
				node->sendControlToDrone(node->hoverCommand);
				return false;
			}

			// i have failed -> try again.
			nextUp = !nextUp;
			node->publishCommand("p reset");
			stageStarted = getMS();
			stage = WAIT_FOR_FIRST;
			node->sendControlToDrone(node->hoverCommand);
			return false;
		case DONE:
			node->sendControlToDrone(controller->update(statePtr));
			return true;
		default:
			return false;
		}
		return false;	// again: should never happen....
	}
}
