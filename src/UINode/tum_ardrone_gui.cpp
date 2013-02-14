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
#include "time.h"
#include "../HelperFunctions.h"

#include "ros/ros.h"
#include "ros/package.h"

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

int getdirtxt (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
    	std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string f = dirp->d_name;
    	if(f.size() > 4 && f.substr(f.size()-4) == ".txt")
    		files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}


tum_ardrone_gui::tum_ardrone_gui(QWidget *parent)
    : QWidget(parent)
{
	ui.setupUi(this);
	rosThread = NULL;
	sensGaz = sensYaw = sensRP = 1;
	currentControlSource = CONTROL_NONE;
	useHovering = true;

	for(int i=0;i<8;i++)
	{
		isPressed[i] = false;
		lastRepeat[i] = 0;
	}


    QObject::connect( this, SIGNAL( setCountsSignal(unsigned int,unsigned int,unsigned int,unsigned int) ),
    		           this, SLOT( setCountsSlot(unsigned int,unsigned int,unsigned int,unsigned int) ) );

    QObject::connect( this, SIGNAL( setPingsSignal(int, int) ),
    		           this, SLOT( setPingsSlot(int, int) ) );

    QObject::connect( this, SIGNAL( setControlSourceSignal(int) ),
    		           this, SLOT( setControlSourceSlot(int) ) );

    QObject::connect( this, SIGNAL( addLogLineSignal(QString) ),
    		           this, SLOT( addLogLineSlot(QString) ) );

    QObject::connect( this, SIGNAL( setAutopilotInfoSignal(QString) ),
    		           this, SLOT( setAutopilotInfoSlot(QString) ) );

    QObject::connect( this, SIGNAL( setStateestimationInfoSignal(QString) ),
    		           this, SLOT( setStateestimationInfoSlot(QString) ) );

    QObject::connect( this, SIGNAL( setMotorSpeedsSignal(QString) ),
    		           this, SLOT( setMotorSpeedsSlot(QString) ) );

    QObject::connect( this, SIGNAL( closeWindowSignal() ),
    		           this, SLOT( closeWindowSlot() ) );


    std::vector<std::string> files = std::vector<std::string>();
    getdirtxt(	ros::package::getPath("tum_ardrone") + std::string("/flightPlans/"),files);

    ui.comboBoxLoadFile->addItem(QString(""), QVariant());
    for(unsigned int i=0;i<files.size();i++)
    	ui.comboBoxLoadFile->addItem(QString(files[i].c_str()), QVariant());

}



tum_ardrone_gui::~tum_ardrone_gui()
{
}

// clicked functions
void tum_ardrone_gui::LandClicked()
{
	rosThread->sendLand();
}
void tum_ardrone_gui::TakeoffClicked()
{
	rosThread->sendTakeoff();
}
void tum_ardrone_gui::ToggleCamClicked()
{
	rosThread->sendToggleCam();
}
void tum_ardrone_gui::FlattrimClicked()
{
	rosThread->sendFlattrim();
}
void tum_ardrone_gui::EmergencyClicked()
{
	rosThread->sendToggleState();
}

void tum_ardrone_gui::ClearClicked()
{
	rosThread->publishCommand("c clearCommands");
}
void tum_ardrone_gui::SendClicked()
{
	QStringList l = ui.plainTextEditSendCommand->toPlainText().split('\n');
	for(int i=0;i<l.length();i++)
	{
		std::string s = l[i].trimmed().toStdString();

		if(s.size() > 0)
			rosThread->publishCommand(std::string("c ")+s);
	}
	setControlSource(CONTROL_AUTO);
}
void tum_ardrone_gui::ClearSendClicked()
{
	ClearClicked();
	SendClicked();
}
void tum_ardrone_gui::ResetClicked()
{
	setControlSource(CONTROL_NONE);
	ClearClicked();
	rosThread->publishCommand("f reset");
}


void tum_ardrone_gui::LoadFileChanged(QString val)
{
	if(val == "")
		ui.plainTextEditSendCommand->setPlainText("");
	else
	{
		std::string path = ros::package::getPath("tum_ardrone") + std::string("/flightPlans/") + val.toStdString();
		addLogLine("Load File "+ path);

		std::ifstream t;
		t.open(path.c_str());
		std::string buffer = "";
		std::string line;
		while(!t.eof())
		{
			std::getline(t, line);
			buffer = buffer + line + "\n";
		}
		t.close();

		ui.plainTextEditSendCommand->setPlainText(buffer.c_str());
	}
}
void tum_ardrone_gui::ToggledUseHovering(int val)
{
	useHovering = (val != 0);
}

void tum_ardrone_gui::ToggledPingDrone(int val)
{
	pingThread->measure = (val != 0);
}

// change control source functions
void tum_ardrone_gui::ControlSourceChanged()
{
	ControlSource s = CONTROL_NONE;

	if(ui.radioButtonControKB->isChecked())
		s = CONTROL_KB;
	if(ui.radioButtonControlNone->isChecked())
		s = CONTROL_NONE;
	if(ui.radioButtonControlJoy->isChecked())
		s = CONTROL_JOY;
	if(ui.radioButtonControlAuto->isChecked())
		s = CONTROL_AUTO;

	if(s != CONTROL_AUTO)
		rosThread->publishCommand("c stop");
	else
		rosThread->publishCommand("c start");

	currentControlSource = s;
}


void tum_ardrone_gui::setControlSourceSlot(int cont)
{

	currentControlSource = (ControlSource)cont;
	if(cont == CONTROL_KB)
		ui.radioButtonControKB->setChecked(true);
	if(cont == CONTROL_NONE)
		ui.radioButtonControlNone->setChecked(true);
	if(cont == CONTROL_JOY)
		ui.radioButtonControlJoy->setChecked(true);
	if(cont == CONTROL_AUTO)
		ui.radioButtonControlAuto->setChecked(true);

	ControlSourceChanged();
}

void tum_ardrone_gui::setCountsSlot(unsigned int nav,unsigned int control,unsigned int pose,unsigned int joy)
{
	char buf[100];
	snprintf(buf,100, "Drone Control: %d Hz", control);
	ui.labelControl->setText(buf);

	snprintf(buf,100, "Joy Input: %d Hz", joy);
	ui.labelJoy->setText(buf);

	snprintf(buf,100, "Drone Navdata: %d Hz", nav);
	ui.labelNavdata->setText(buf);

	snprintf(buf,100, "Pose Estimates: %d Hz", pose);
	ui.labelPoseEst->setText(buf);
}

void tum_ardrone_gui::setPingsSlot(int p500, int p20000)
{
	char buf[100];
	snprintf(buf,100, "Pings (RTT): %d (500B), %d (20kB)", p500, p20000);
	ui.labelDronePings->setText(buf);
}

void tum_ardrone_gui::addLogLineSlot(QString s)
{
	ui.plainTextEditMessages->appendPlainText(s);
}
void tum_ardrone_gui::setAutopilotInfoSlot(QString s)
{
	ui.plainTextEditAutopilotStatus->setPlainText(s);
}
void tum_ardrone_gui::setStateestimationInfoSlot(QString s)
{
	ui.plainTextEditStateestimationStatus->setPlainText(s);
}
void tum_ardrone_gui::setMotorSpeedsSlot(QString s)
{
	ui.labelDroneMotors->setText(s);
}
void tum_ardrone_gui::closeWindowSlot()
{
	closeWindow();
}


// these may be called from external thread,
// so they just "forward" the request.
void tum_ardrone_gui::setCounts(unsigned int nav,unsigned int control,unsigned int pose,unsigned int joy)
{
	emit setCountsSignal(nav, control, pose, joy);
}
void tum_ardrone_gui::setControlSource(ControlSource cont)
{
	emit setControlSourceSignal((int)cont);
}
void tum_ardrone_gui::addLogLine(std::string s)
{
	emit addLogLineSignal(QString(s.c_str()));
}
void tum_ardrone_gui::setAutopilotInfo(std::string s)
{
	emit setAutopilotInfoSignal(QString(s.c_str()));
}
void tum_ardrone_gui::setMotorSpeeds(std::string s)
{
	emit setMotorSpeedsSignal(QString(s.c_str()));
}
void tum_ardrone_gui::setStateestimationInfo(std::string s)
{
	emit setStateestimationInfoSignal(QString(s.c_str()));
}
void tum_ardrone_gui::setPings(int p500, int p20000)
{
	emit setPingsSignal(p500, p20000);
}
void tum_ardrone_gui::closeWindow()
{
	emit closeWindowSignal();
}


// KB control stuff
int tum_ardrone_gui::mapKey(int k)
{
	switch(k)
	{
		case 74: //j
			return 0;
		case 75: //k
			return 1;
		case 76: //l
			return 2;
		case 73: //i
			return 3;
		case 85: //u
			return 4;
		case 79: //o
			return 5;
		case 81: //q
			return 6;
		case 65: //a
			return 7;
	}
	return -1;
}

void tum_ardrone_gui::keyReleaseEvent( QKeyEvent * key)
{
	if(currentControlSource == CONTROL_KB)
	{
		int idx = mapKey(key->key());
		if(idx >= 0)
		{
			bool changed = false;
			if(!key->isAutoRepeat())	// ignore autorepeat-releases (!)
			{
				changed = isPressed[idx];
				isPressed[idx] = false;
			}

			if(changed)
				rosThread->sendControlToDrone(calcKBControl());
		}
	}
}

void tum_ardrone_gui::keyPressEvent( QKeyEvent * key)
{

	if(currentControlSource == CONTROL_KB)
	{
		int idx = mapKey(key->key());
		if(idx >= 0)
		{
			bool changed = !isPressed[idx];

			isPressed[idx] = true;
			lastRepeat[idx] = getMS();

			if(changed)
				rosThread->sendControlToDrone(calcKBControl());
		}

		else if(key->key() == 83)	// s
			rosThread->sendTakeoff();

		else if(key->key() == 68)	// d
			rosThread->sendLand();
	}


	if(key->key() == 16777216)	// ESC
	{
		setFocus();
		setControlSource(CONTROL_KB);
	}


	if(key->key() == 16777264)	// F1
	{
		rosThread->sendToggleState();
	}
}

ControlCommand tum_ardrone_gui::calcKBControl()
{
	// clear keys that have not been refreshed for 1s, it is set to "not pressed"
	for(int i=0;i<8;i++)
		isPressed[i] = isPressed[i] && ((lastRepeat[i] + 1000) > getMS());

	ControlCommand c;

	if(isPressed[0]) c.roll = -sensRP; // j
	if(isPressed[1]) c.pitch = sensRP; // k
	if(isPressed[2]) c.roll = sensRP; // l
	if(isPressed[3]) c.pitch = -sensRP; // i
	if(isPressed[4]) c.yaw = -sensYaw; // u
	if(isPressed[5]) c.yaw = sensYaw; // o
	if(isPressed[6]) c.gaz = sensRP; // q
	if(isPressed[7]) c.gaz = -sensRP; // a

	return c;
}
