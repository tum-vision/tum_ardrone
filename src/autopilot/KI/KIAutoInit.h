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
#ifndef __KIAUTOINIT_H
#define __KIAUTOINIT_H

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
	int reachHeightMS;
	float controlCommandMultiplier;
public:
	KIAutoInit(bool resetMap = true, int imoveTimeMS=500, int iwaitTimeMS=800, int reachHeightMS=6000, float controlMult = 1.0, bool takeoff=true);
	~KIAutoInit(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIAUTOINIT_H */
