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
 
#include "MapView.h"
#include "../HelperFunctions.h"
#include <cvd/gl_helpers.h>
#include <gvars3/GStringUtil.h>
#include "Predictor.h"
#include <gvars3/instances.h>
#include "DroneKalmanFilter.h"
#include "PTAMWrapper.h"
#include "EstimationNode.h"

pthread_mutex_t MapView::trailPointsVec_CS = PTHREAD_MUTEX_INITIALIZER; //pthread_mutex_lock( &cs_mutex );

MapView::MapView(DroneKalmanFilter* f, PTAMWrapper* p, EstimationNode* nde)
{
	filter = f;
	ptamWrapper = p;
	node = nde;
	drawUI = UI_PRES;
	resetRequested = false;
	trailPoints = std::vector<TrailPoint>();
	predConvert = new Predictor();
	clearTrail = false;
	resetMapViewFlag = true;

}

void MapView::ResetInternal()
{
	trailPoints.clear();
}


MapView::~MapView(void)
{
	delete predConvert;

}

void MapView::startSystem()
{
	keepRunning = true;
	changeSizeNextRender = false;

	start();
}

void MapView::stopSystem()
{
	keepRunning = false;
	join();
}


void MapView::run()
{
	sleep(1000);
    myGLWindow = new GLWindow2(CVD::ImageRef(640,480), "PTAM Drone Map View",this);
	myGLWindow->set_title("PTAM Drone Map View");

	while(keepRunning)
	{
		Render();
		usleep(30000);	// TODO: some more advanced than just render at 33fps
	}

	delete myGLWindow;
}


void MapView::Render()
{

	// get new pose.
	pthread_mutex_lock(&filter->filter_CS);
	lastFramePoseSpeed = filter->getCurrentPoseSpeedAsVec();	// Note: this is maybe an old pose, but max. one frame old = 50ms = not noticable.
	pthread_mutex_unlock(&filter->filter_CS);

	


	if(clearTrail)
	{
		trailPoints.clear();
		clearTrail = false;
	}

	// render
	bool addTrail;
	if(trailPoints.size() == 0)
		addTrail = true;
	else
	{
		TooN::Vector<3> distToLast = lastFramePoseSpeed.slice<0,3>() - trailPoints[trailPoints.size()-1].pointFilter;
		double d = distToLast[0]*distToLast[0] + distToLast[1]*distToLast[1]+distToLast[2]*distToLast[2];
		addTrail = d > 0.1*0.1;
	}


	


	// the following complicated code is to save trail-points in ptam-scale, such that scale-reestimation will re-scale the drawn path.
	if(addTrail)
	{
		if(ptamWrapper->PTAMStatus == ptamWrapper->PTAM_BEST ||
				ptamWrapper->PTAMStatus == ptamWrapper->PTAM_TOOKKF ||
				ptamWrapper->PTAMStatus == ptamWrapper->PTAM_GOOD)
		{
			if(ptamWrapper->PTAMInitializedClock != 0 && getMS() - ptamWrapper->PTAMInitializedClock > 200)
			{
				TooN::Vector<3> PTAMScales = filter->getCurrentScales();
				TooN::Vector<3> PTAMOffsets = filter->getCurrentOffsets().slice<0,3>();

				TooN::Vector<3> ptamPointPos = lastFramePoseSpeed.slice<0,3>();
				ptamPointPos -= PTAMOffsets;
				ptamPointPos /= PTAMScales[0];

				trailPoints.push_back(TrailPoint(
					lastFramePoseSpeed.slice<0,3>(),
					ptamPointPos
				));
			}
		}
		else if(ptamWrapper->PTAMStatus == ptamWrapper->PTAM_LOST ||
				ptamWrapper->PTAMStatus == ptamWrapper->PTAM_FALSEPOSITIVE)
		{
			if(ptamWrapper->PTAMInitializedClock != 0 && getMS() - ptamWrapper->PTAMInitializedClock > 200)
			{
				TooN::Vector<3> PTAMScales = filter->getCurrentScales();
				TooN::Vector<3> PTAMOffsets = filter->getCurrentOffsets().slice<0,3>();

				TooN::Vector<3> ptamPointPos = lastFramePoseSpeed.slice<0,3>();
				ptamPointPos -= PTAMOffsets;
				ptamPointPos /= PTAMScales[0];

				trailPoints.push_back(TrailPoint(
					lastFramePoseSpeed.slice<0,3>(),
					ptamPointPos
				));
			}
		}
		else
		{
			trailPoints.push_back(TrailPoint(
				lastFramePoseSpeed.slice<0,3>()
			));
		}
	}





	if(resetMapViewFlag)
	{
		resetMapView();
		resetMapViewFlag = false;
	}

	// get lineWidthFactor
	lineWidthFactor = sqrt((float)(myGLWindow->size()[0] * myGLWindow->size()[1] / (640*480)));



	plotGrid();

	pthread_mutex_lock(&ptamWrapper->shallowMapCS);
	std::vector<tse3>* kfl = &(ptamWrapper->keyFramesTransformed);
	
	// draw keyframes
	for(unsigned int i=0;i<kfl->size();i++)
	{
		plotCam((*kfl)[i],false,2,0.04f,1);
	}
	
	// draw trail
	drawTrail();

	// draw keypoints
	plotMapPoints();
	pthread_mutex_unlock(&ptamWrapper->shallowMapCS);

	// draw predicted cam


	// real in opaque
	predConvert->setPosRPY(lastFramePoseSpeed[0], lastFramePoseSpeed[1], lastFramePoseSpeed[2], lastFramePoseSpeed[3], lastFramePoseSpeed[4], lastFramePoseSpeed[5]);
	plotCam(predConvert->droneToGlobal,true,5.0f,0.2f,1);


	// --------------------- make msg ------------------------------
	msg = "";
	TooN::Vector<6> of = filter->getCurrentOffsets();
	TooN::Vector<3> sc = filter->getCurrentScales();


	if(drawUI == UI_DEBUG)
	{
		snprintf(charBuf,1000,"Pose:              ");
		snprintf(charBuf+10,800, "x: %.2f                          ",lastFramePoseSpeed[0]);
		snprintf(charBuf+20,800, "y: %.2f                          ",lastFramePoseSpeed[1]);
		snprintf(charBuf+30,800, "z: %.2f                          ",lastFramePoseSpeed[2]);
		snprintf(charBuf+40,800, "r: %.2f                          ",lastFramePoseSpeed[3]);
		snprintf(charBuf+50,800, "p: %.2f                          ",lastFramePoseSpeed[4]);
		snprintf(charBuf+60,800, "y: %.2f                          ",lastFramePoseSpeed[5]);
		snprintf(charBuf+70,800, "vx: %.2f                          ",lastFramePoseSpeed[6]);
		snprintf(charBuf+80,800, "vy: %.2f                          ",lastFramePoseSpeed[7]);
		snprintf(charBuf+90,800, "vz: %.2f                          ",lastFramePoseSpeed[8]);
		snprintf(charBuf+100,800, "vy: %.2f",lastFramePoseSpeed[9]);
		msg += charBuf;
	

		snprintf(charBuf,1000,"\nSync:              ");
		snprintf(charBuf+10,800, "ox: %.2f                          ",of[0]);
		snprintf(charBuf+20,800, "oy: %.2f                          ",of[1]);
		snprintf(charBuf+30,800, "oz: %.2f                          ",of[2]);
		snprintf(charBuf+40,800, "or: %.2f                          ",of[3]);
		snprintf(charBuf+50,800, "op: %.2f                          ",of[4]);
		snprintf(charBuf+60,800, "oy: %.2f                          ",of[5]);
		snprintf(charBuf+70,800, "Sx: %.2f                          ",sc[0]);
		snprintf(charBuf+80,800, "Sy: %.2f                          ",sc[1]);
		snprintf(charBuf+90,800, "Sz: %.2f",sc[2]);
		msg += charBuf;


		snprintf(charBuf,1000,"\nStDvs:                 ");
		snprintf(charBuf+10,800, "x: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[0]));
		snprintf(charBuf+20,800, "y: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[1]));
		snprintf(charBuf+30,800, "z: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[2]));
		snprintf(charBuf+40,800, "r: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[3]));
		snprintf(charBuf+50,800, "p: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[4]));
		snprintf(charBuf+60,800, "y: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[5]));
		snprintf(charBuf+70,800, "vx: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[6]));
		snprintf(charBuf+80,800, "vy: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[7]));
		snprintf(charBuf+90,800, "vz: %.2f                          ",std::sqrt((double)lastFramePoseSpeed[8]));
		snprintf(charBuf+100,800, "vy: %.2f",std::sqrt((double)lastFramePoseSpeed[9]));
		msg += charBuf;
	}
	else
	{
		snprintf(charBuf,1000,"Drone Pose:             ");
		snprintf(charBuf+13,800, "xyz=(%.2f,                          ",lastFramePoseSpeed[0]);
		snprintf(charBuf+25,800, "%.2f,                          ",lastFramePoseSpeed[1]);
		snprintf(charBuf+32,800, "%.2f),                      ",lastFramePoseSpeed[2]);
		snprintf(charBuf+42,800, "rpy=(%.2f,                 ",lastFramePoseSpeed[3]);
		snprintf(charBuf+54,800, "%.2f,                          ",lastFramePoseSpeed[4]);
		snprintf(charBuf+61,800, "%.2f)                          ",lastFramePoseSpeed[5]);
		msg += charBuf;
	}
	

	myGLWindow->GetMousePoseUpdate();



	CVD::glSetFont("sans");
	if(drawUI != UI_NONE)
		myGLWindow->DrawCaption(msg);

	if(drawUI == UI_DEBUG)
	{
	  glMatrixMode(GL_PROJECTION);
	  glPushMatrix();
	  glTranslatef((float)0, (float)100, 0.0);
	  glScalef(45,-45,1);
	  snprintf(charBuf,1000,"xyz: %.2f %.2f %.2f",lastFramePoseSpeed[0],lastFramePoseSpeed[1],lastFramePoseSpeed[2]);
	  CVD::glDrawText(charBuf, CVD::NICE, 1.6, 0.1);
	  glPopMatrix();
	}
	myGLWindow->swap_buffers();
	myGLWindow->HandlePendingEvents();

}


void MapView::drawTrail()
{
	glEnable(GL_DEPTH_TEST);

	// draw cam
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity();
	glScaled(0.1,0.1,0.1);
	CVD::glMultMatrix(mse3ViewerFromWorld);
	SetupFrustum();
	glEnable(GL_BLEND); 
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(3*lineWidthFactor);
	glBegin(GL_LINES);
	glColor4f(0,1,0,0.6);

	TooN::Vector<3> PTAMScales = filter->getCurrentScales();
	TooN::Vector<3> PTAMOffsets = filter->getCurrentOffsets().slice<0,3>();

	for(unsigned int i=1;i<trailPoints.size();i++)
	{
		if(trailPoints[i].PTAMValid)
		{
			trailPoints[i].pointFilter = trailPoints[i].pointPTAM;
			trailPoints[i].pointFilter[0] *= PTAMScales[0];
			trailPoints[i].pointFilter[1] *= PTAMScales[1];
			trailPoints[i].pointFilter[2] *= PTAMScales[2];
			trailPoints[i].pointFilter += PTAMOffsets;
		}
		if(i > 1 && i < trailPoints.size()-1)
			glVertex3f((float)trailPoints[i].pointFilter[0], (float)trailPoints[i].pointFilter[1], (float)trailPoints[i].pointFilter[2]);
		glVertex3f((float)trailPoints[i].pointFilter[0], (float)trailPoints[i].pointFilter[1], (float)trailPoints[i].pointFilter[2]);
	}

	glEnd();


	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void MapView::plotMapPoints()
{
	
	glEnable(GL_DEPTH_TEST);

	// draw cam
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity();
	glScaled(0.1,0.1,0.1);
	CVD::glMultMatrix(mse3ViewerFromWorld);
	SetupFrustum();
	float width = 1.0f;
	float len = 0.02f;

	glLineWidth(width*lineWidthFactor);
	glBegin(GL_LINES);
	glColor3f(1,0,0);

	std::vector<tvec3>* mpl = &(ptamWrapper->mapPointsTransformed);
	
	for(unsigned int i=0;i<mpl->size();i++)
	{
		TooN::Vector<3> pos = (*mpl)[i];
		
		glVertex3f((float)pos[0]-len, (float)pos[1], (float)pos[2]);
		glVertex3f((float)pos[0]+len, (float)pos[1], (float)pos[2]);		
		glVertex3f((float)pos[0], (float)pos[1]-len, (float)pos[2]);
		glVertex3f((float)pos[0], (float)pos[1]+len, (float)pos[2]);		
		glVertex3f((float)pos[0], (float)pos[1], (float)pos[2]-len);
		glVertex3f((float)pos[0], (float)pos[1], (float)pos[2]+len);
	}

	glEnd();


	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
}

void MapView::plotCam(TooN::SE3<> droneToGlobal, bool xyCross, float thick, float len, float alpha)
{
	glEnable(GL_DEPTH_TEST);

	// draw cam
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity();
	glScaled(0.1,0.1,0.1);
	CVD::glMultMatrix(mse3ViewerFromWorld * droneToGlobal);
	SetupFrustum();


	glLineWidth(thick*lineWidthFactor);

	if(alpha < 1)
	{
		glEnable(GL_BLEND); 
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
		glDisable(GL_BLEND); 


	glBegin(GL_LINES);
	glColor4f(1,0,0,alpha);
	glVertex3f(-len, 0.0f, 0.0f);
	glVertex3f(len, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, -len, 0.0f);

	glColor4f(0,1,0,alpha);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, len, 0.0f);

	glColor4f(1,1,1,alpha);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, len);
	glEnd();

	
	if(xyCross)
	{
		glLineWidth(1*lineWidthFactor);
		glColor4f(1,1,1, alpha);
		SetupModelView();
		TooN::Vector<2> v2CamPosXY = droneToGlobal.get_translation().slice<0,2>();
		glBegin(GL_LINES);
		glColor4f(1,1,1, alpha);
		glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
		glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
		glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
		glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
		glEnd();
	}
	glDisable(GL_BLEND); 
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void MapView::resetMapView()
{	
	mse3ViewerFromWorld =  TooN::SE3<>::exp(TooN::makeVector(0,0,4,0,0,0)) * TooN::SE3<>::exp(TooN::makeVector(0,0,0,0.8 * M_PI,0,0));
}

void MapView::plotGrid()
{
	// Update viewer position according to mouse input:

 	std::pair<TooN::Vector<6>, TooN::Vector<6> > pv6 = myGLWindow->GetMousePoseUpdate();
	TooN::SE3<> se3CamFromMC;
	se3CamFromMC.get_translation() = mse3ViewerFromWorld * TooN::makeVector(0,0,0);
	mse3ViewerFromWorld = TooN::SE3<>::exp(pv6.first) * 
	se3CamFromMC * TooN::SE3<>::exp(pv6.second).inverse() * se3CamFromMC.inverse() * mse3ViewerFromWorld;

	myGLWindow->SetupViewport();
	glClearColor(0,0,0,0);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glEnable(GL_POINT_SMOOTH);
	//glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColorMask(1,1,1,1);
	glDisable(GL_DEPTH_TEST);

	// draw grid
	SetupFrustum();
	SetupModelView();
	glLineWidth(1*lineWidthFactor);

	double gridBigSpacing = 1;
	double gridSmallSpacing = 0.2;
	double gridBigMax = 30;
	double gridSmallMax = 5;

	// fine grid
	glBegin(GL_LINES);
	for(double x=-gridSmallMax; x<=gridSmallMax;x+=gridSmallSpacing)
	{
		if(x != 0 && (x-floor(x)) > 0.1)
		{
			glColor3f(0.2f,0.2f,0.2f);
			glVertex3d(x, -gridSmallMax, 0.0);
			glVertex3d(x, gridSmallMax, 0.0);
		}
	}
	for(double y=-gridSmallMax; y<=gridSmallMax;y+=gridSmallSpacing)
	{
		if(y != 0 && (y-floor(y)) > 0.1)
		{
			glColor3f(0.25f,0.25f,0.25f);
			glVertex3d(-gridSmallMax,y, 0.0);
			glVertex3d(gridSmallMax,y, 0.0);
		}
	}
	glEnd();


	//big grid
	glLineWidth(2*lineWidthFactor);
	glBegin(GL_LINES);
	for(double x=-gridBigMax; x<=gridBigMax;x+=gridBigSpacing)
	{
		if(x != 0)
		{
			glColor3f(0.6f,0.6f,0.6f);
			glVertex3d(x, -gridBigMax, 0.0);
			glVertex3d(x, gridBigMax, 0.0);
		}
	}
	for(double y=-gridBigMax; y<=gridBigMax;y+=gridBigSpacing)
	{
		if(y != 0)
		{
			glColor3f(0.6f,0.6f,0.6f);
			glVertex3d(-gridBigMax,y, 0.0);
			glVertex3d(gridBigMax,y, 0.0);
		}
	}
	glEnd();


	//xy lines
	glLineWidth(2.5*lineWidthFactor);
	glBegin(GL_LINES);

	glColor3f(1,1,1);
	glVertex3d(-gridBigMax,0, 0.0);
	glVertex3d(gridBigMax,0, 0.0);
	glVertex3d(0,-gridBigMax, 0.0);
	glVertex3d(0,gridBigMax, 0.0);

	// colored xy lines
	glColor3f(1,0,0);
	glVertex3d(0,0,0);
	glVertex3d(1,0,0);
	glColor3f(0,1,0);
	glVertex3d(0,0,0);
	glVertex3d(0,1,0);
	glColor3f(1,1,1);
	glVertex3d(0,0,0);
	glVertex3d(0,0,1);
	glEnd();



}

void MapView::SetupFrustum()
{
  glMatrixMode(GL_PROJECTION);  
  glLoadIdentity();
  double zNear = 0.03;
  glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,50);
  glScalef(1,1,-1);
  return;
};

void MapView::SetupModelView(TooN::SE3<> se3WorldFromCurrent)
{
  glMatrixMode(GL_MODELVIEW);  
  glLoadIdentity();
  CVD::glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
  return;
};

void MapView::on_key_down(int key)
{
	if(key == 114) // r
	{
		node->publishCommand("f reset");
	}
	if(key == 117) // u
	{
		node->publishCommand("m toggleUI");
	}
	if(key == 118)	// v
	{
		node->publishCommand("m resetView");
	}
	if(key == 108) // l
	{
		node->publishCommand("toggleLog");
	}
	if(key == 116) // t
	{
		node->publishCommand("m clearTrail");
	}
}


// handles commands with "m" or "f" prefix.
// called by external thread, so watch sync.
bool MapView::handleCommand(std::string s)
{
	if(s.length() == 8 && s.substr(0,8) == "toggleUI")
	{
		if(drawUI == UI_NONE) drawUI = UI_DEBUG;
		else if(drawUI == UI_DEBUG) drawUI = UI_PRES;
		else if(drawUI == UI_PRES) drawUI = UI_NONE;
	}
	if(s.length() == 9 && s.substr(0,9) == "resetView")
	{
		resetMapViewFlag = true;
	}
	if(s.length() == 10 && s.substr(0,10) == "clearTrail")
	{
		clearTrail = true;
	}
	// f reset:
	// resets filter pos and PTAM.
	if(s.length() == 5 && s.substr(0,5) == "reset")
	{
		filter->reset();
		ptamWrapper->Reset();
		clearTrail = true;
	}
	return true;
}
