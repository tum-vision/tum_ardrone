#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 

#include "../DroneController.h"
#include "tum_ardrone/filter_state.h"

class ControlNode;
class DroneController;


class KIProcedure
{
protected:
	ControlNode* node;
	DroneController* controller;

public:
	std::string command;

	// called externally before first call to update().
	inline void setPointers(ControlNode* node, DroneController* cont)
	{
		this->node = node;
		controller = cont;
	}

	// is called with control-frequency, is supposed to each time generate and send a new
	// control command to the drone.
	// returns wether the goal of this KI has been reached (leads to the KI being destroyed and the next one being popped).
	virtual bool update(const tum_ardrone::filter_stateConstPtr statePtr) = 0;

	// constructed shortly before first update.
	inline KIProcedure(void) {node = NULL; controller = NULL; command = "not set"; };
	inline ~KIProcedure(void) {};
};

