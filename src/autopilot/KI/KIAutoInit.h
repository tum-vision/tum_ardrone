#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */

#include "KIProcedure.h"

class KIAutoInit : public KIProcedure
{
private:
	enum {NONE, STARTED, WAIT_FOR_FIRST, TOOK_FIRST, WAIT_FOR_SECOND, DONE} stage;
	int stageStarted;
	bool nextUp;
	bool resetMap;
	int moveTimeMS;
	int waitTimeMS;
public:
	KIAutoInit(bool resetMap = true, int imoveTimeMS=500, int iwaitTimeMS=800);
	~KIAutoInit(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

