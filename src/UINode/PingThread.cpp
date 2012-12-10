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
 
#include "PingThread.h"
#include "cvd/thread.h"
#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include "RosThread.h"
#include "tum_ardrone_gui.h"

PingThread::PingThread()
{
	line1[0] = '\0';
	line2[0] = '\0';
	keepRunning = true;
	measure = true;

	p500 = 25;
	p20000 = 50;

	rosThread = NULL;
	gui = NULL;
}

PingThread::~PingThread(void)
{


}

void PingThread::startSystem()
{
	keepRunning = true;
	start();
}

void PingThread::stopSystem()
{
	keepRunning = false;
	join();
}

double parsePingResult(std::string s)
{
	// 20008 bytes from localhost (127.0.0.1): icmp_req=1 ttl=64 time=0.075 ms
	int pos = s.find("time=");
	int found = 0;
	float ms;
	if(pos != std::string::npos)
		found = sscanf(s.substr(pos).c_str(),"time=%f",&ms);

	if(found == 1 && pos != std::string::npos)
		return ms;
	else
		return 10000;
}

void PingThread::run()
{
	std::cout << "Starting PING Thread" << std::endl;

	sprintf(pingCommand20000,"ping -c 1 -s 20000 -w 1 192.168.1.1");
	sprintf(pingCommand500,"ping -c 1 -s 500 -w 1 192.168.1.1");
	ros::Rate r(2.0);
	FILE *p;

	while(keepRunning)
	{
		if(measure)
		{
			// ping twice, with a sleep in between
			p = popen(pingCommand500,"r");
			fgets(line1, 200, p);
			fgets(line1, 200, p);
			pclose(p);

			// sleep 1s
			r.sleep();
			if(!keepRunning) break;
			r.sleep();
			if(!keepRunning) break;



			p = popen(pingCommand20000,"r");
			fgets(line2, 200, p);
			fgets(line2, 200, p);
			pclose(p);


			// parse results which should be in line1 and line2
			double res500 = parsePingResult(line1);
			double res20000 = parsePingResult(line2);

			std::cout << "new ping values: 500->" << res500 << " 20000->" << res20000 << std::endl;

			// clip between 10 and 1000.
			res500 = std::min(1000.0,std::max(10.0,res500));
			res20000 = std::min(1000.0,std::max(10.0,res20000));

			// update
			p500 = 0.7 * p500 + 0.3 * res500;
			p20000 = 0.7 * p20000 + 0.3 * res20000;

			// send
			snprintf(line1,200,"pings %d %d", (int)p500, (int)p20000);
			if(rosThread != NULL) rosThread->publishCommand(line1);
			if(gui != NULL) gui->setPings((int)p500, (int)p20000);

			// sleep 1s
			r.sleep();
			if(!keepRunning) break;
			r.sleep();
			if(!keepRunning) break;
		}
		else
		{
			r.sleep();
			if(!keepRunning) break;
			r.sleep();
			if(!keepRunning) break;
			r.sleep();
			if(!keepRunning) break;
			r.sleep();
			if(!keepRunning) break;

			// send
			snprintf(line1,200,"pings %d %d", (int)p500Default, (int)p20000Default);
			if(rosThread != NULL) rosThread->publishCommand(line1);
			if(gui != NULL) gui->setPings((int)p500Default, (int)p20000Default);
		}
	}

	std::cout << "Exiting PING Thread" << std::endl;
}
