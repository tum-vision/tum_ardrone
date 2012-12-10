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
#ifndef __MAPVIEW_H
#define __MAPVIEW_H
 
 
 

#include "GLWindow2.h"
#include <deque>
#include "cvd/thread.h"
#include "TooN/se3.h"
#include "MouseKeyHandler.h"


class DroneKalmanFilter;
class PTAMWrapper;
class Predictor;
class EstimationNode;

class TrailPoint
{
public:
	inline TrailPoint(TooN::Vector<3> filter)
	{
		pointFilter = filter;
		PTAMValid = false;
	}
	inline TrailPoint(TooN::Vector<3> filter, TooN::Vector<3> ptam)
	{
		pointFilter = filter;
		pointPTAM = ptam;
		PTAMValid = true;
	}
	TooN::Vector<3> pointPTAM;
	TooN::Vector<3> pointFilter;
	bool PTAMValid;
};

class MapView : private CVD::Thread, private MouseKeyHandler
{
private:
	// base window
	GLWindow2* myGLWindow;
	CVD::ImageRef desiredWindowSize;		// size the window scould get changed to if [changeSizeNextRender]
	CVD::ImageRef defaultWindowSize;		// size the window gets opened with
	bool changeSizeNextRender;


	// the associated thread's run function.
	// calls control() every time a new PTAM info is available or every 20ms.
	void run();

	// main routine; uses all available information, 
	// in order to calculate and send a new control command to the drone
	void control();

	// renders map view
	void Render();

	DroneKalmanFilter* filter;
	PTAMWrapper* ptamWrapper;
	EstimationNode* node;

	bool resetRequested;

	// keep Running
	bool keepRunning;

	Predictor* predConvert;


	// ---------- rendering stuff ---------------------------
	char charBuf[1000];
	std::string msg;
	enum {UI_NONE = 0, UI_DEBUG = 1, UI_PRES = 2} drawUI;
	float lineWidthFactor;

	// plot stuff
	void plotMapPoints();
	void plotGrid();
	void plotKeyframes();
	void SetupFrustum();
	void SetupModelView(TooN::SE3<> se3WorldFromCurrent = TooN::SE3<>());

	// viewing options
	TooN::SE3<> mse3ViewerFromWorld;
	TooN::Vector<3> mv3MassCenter;
	bool resetMapViewFlag;

	void plotCam(TooN::SE3<> droneToGlobal, bool xyCross, float thick, float len, float alpha);
	void drawTrail();

	// resets tracking. private as it needs to be called from internal thread.
	void ResetInternal();

	// values for rendering.
	TooN::Vector<10> lastFramePoseSpeed;
	bool inControl;
	bool clearTrail;


	// trail for rendering
	std::vector<TrailPoint> trailPoints;

public:

	MapView(DroneKalmanFilter* f, PTAMWrapper* p, EstimationNode* nde);
	~MapView(void);

	bool handleCommand(std::string s);

	inline void Reset() {resetRequested = true;};

	// Event handling routines.
	// get called by the myGLWindow on respective event.
	void on_key_down(int key);
	//virtual void on_mouse_move(CVD::ImageRef where, int state);
	//virtual void on_mouse_down(CVD::ImageRef where, int state, int button);
	//virtual void on_event(int event);


	static pthread_mutex_t trailPointsVec_CS; //pthread_mutex_lock( &cs_mutex );

	// start and stop system and respective thread.
	void startSystem();
	void stopSystem();

	void resetMapView();
};
#endif /* __MAPVIEW_H */

