#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 */
 
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

