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
 
#include "tum_ardrone_gui.h"
#include "RosThread.h"
#include "PingThread.h"

#include <QtGui>
#include <QApplication>
#include "ros/ros.h"

// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;

int main(int argc, char *argv[])
{
	std::cout << "Starting drone_gui Node" << std::endl;

	// ROS
	ros::init(argc, argv, "drone_gui");
    RosThread t;
    PingThread p;

    // UI
    QApplication a(argc, argv);
    tum_ardrone_gui w;

    // make them communicate with each other
    t.gui = &w;
    w.rosThread = &t;
    p.gui = &w;
    p.rosThread = &t;
    w.pingThread = &p;

    // start them.
    t.startSystem();
    p.startSystem();
    w.show();

    // wait until windows closed....
    int ec = a.exec();

     // stop ROS again....
    t.stopSystem();
    p.stopSystem();

	std::cout << "Exiting drone_gui Node" << std::endl;

    return ec;
}
