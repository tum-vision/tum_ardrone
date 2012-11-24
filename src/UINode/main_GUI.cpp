/*
 * drone_gui:
 * GUI to manage drone_stateestimation and drone_autopilot (send commands).
 *
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
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
