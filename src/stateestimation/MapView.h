#pragma once
/*
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * 
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 

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

