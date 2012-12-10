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
#ifndef __KIFLYTO_H
#define __KIFLYTO_H
 

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

#endif /* __KIFLYTO_H */
