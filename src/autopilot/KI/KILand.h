#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 

#include "KIProcedure.h"

class KILand : public KIProcedure
{
private:
	bool fresh;
public:
	KILand(void);
	~KILand(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

