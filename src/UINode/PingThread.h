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
#ifndef __PINGTHREAD_H
#define __PINGTHREAD_H
 
 
#include "cvd/thread.h"

class tum_ardrone_gui;
class RosThread;


class PingThread : private CVD::Thread
{
private:
	// the associated thread's run function.
	void run();

	// keep Running
	bool keepRunning;


	// buffers
    char pingCommand500[100];
    char pingCommand20000[100];
    char line1[200];
    char line2[200];


    // running averages
    double p500;
    double p20000;

    static const double p500Default = 25;
    static const double p20000Default = 50;
public:
	PingThread(void);
	~PingThread(void);

	// start and stop system and respective thread.
	// to be called externally
	void startSystem();
	void stopSystem();

	// start and stop pinging
	void setEnabled(bool);

	tum_ardrone_gui* gui;
	RosThread* rosThread;
	bool measure;
};

#endif /* __PINGTHREAD_H */
