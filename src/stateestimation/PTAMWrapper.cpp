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
 
 
 
#include "PTAMWrapper.h"
#include <cvd/gl_helpers.h>
#include <gvars3/instances.h>
#include "PTAM/ATANCamera.h"
#include "PTAM/MapMaker.h"
#include "PTAM/Tracker.h"
#include "PTAM/Map.h"
#include "PTAM/MapPoint.h"
#include "../HelperFunctions.h"
#include "Predictor.h"
#include "DroneKalmanFilter.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "GLWindow2.h"
#include "EstimationNode.h"
#include <iostream>
#include <fstream>
#include <string>

pthread_mutex_t PTAMWrapper::navInfoQueueCS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMWrapper::shallowMapCS = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t PTAMWrapper::logScalePairs_CS = PTHREAD_MUTEX_INITIALIZER;

PTAMWrapper::PTAMWrapper(DroneKalmanFilter* f, EstimationNode* nde)
{
	filter = f;
	node = nde;

	mpMap = 0; 
	mpMapMaker = 0; 
	mpTracker = 0; 
	predConvert = 0;
	predIMUOnlyForScale = 0;
	mpCamera = 0;
	newImageAvailable = false;
	
	mapPointsTransformed = std::vector<tvec3>();
	keyFramesTransformed = std::vector<tse3>();

	
	predConvert = new Predictor();
	predIMUOnlyForScale = new Predictor();
	imuOnlyPred = new Predictor();

	drawUI = UI_PRES;
	frameWidth = frameHeight = 0;

	minKFDist = 0;
	minKFWiggleDist = 0;
	minKFTimeDist = 0;

	maxKF = 60;

	logfileScalePairs = 0;
}

void PTAMWrapper::ResetInternal()
{
	mimFrameBW.resize(CVD::ImageRef(frameWidth, frameHeight));
	mimFrameBW_workingCopy.resize(CVD::ImageRef(frameWidth, frameHeight));


	if(mpMapMaker != 0) delete mpMapMaker;
	if(mpMap != 0) delete mpMap;
	if(mpTracker != 0) delete mpTracker;
	if(mpCamera != 0) delete mpCamera;


	// read camera calibration (yes, its done here)
	std::string file = node->calibFile;
	while(node->arDroneVersion == 0)
	{
		std::cout << "Waiting for first navdata to determine drone version!" << std::endl;
		usleep(250000);
	}
	if(file.size()==0)
	{
		if(node->arDroneVersion == 1)
			file = node->packagePath + "/camcalib/ardrone1_default.txt";
		else if(node->arDroneVersion == 2)
			file = node->packagePath + "/camcalib/ardrone2_default.txt";
	}

	std::ifstream fleH (file.c_str());
	TooN::Vector<5> camPar;
	fleH >> camPar[0] >> camPar[1] >> camPar[2] >> camPar[3] >> camPar[4];
	fleH.close();
	std::cout<< "Set Camera Paramerer to: " << camPar[0] << " " << camPar[1] << " " << camPar[2] << " " << camPar[3] << " " << camPar[4] << std::endl;



	mpMap = new Map;
	mpCamera = new ATANCamera(camPar);
	mpMapMaker = new MapMaker(*mpMap, *mpCamera);
	mpTracker = new Tracker(CVD::ImageRef(frameWidth, frameHeight), *mpCamera, *mpMap, *mpMapMaker);

	setPTAMPars(minKFTimeDist, minKFWiggleDist, minKFDist);

	predConvert->setPosRPY(0,0,0,0,0,0);
	predIMUOnlyForScale->setPosRPY(0,0,0,0,0,0);

	resetPTAMRequested = false;
	forceKF = false;
	isGoodCount = 0;
	lastAnimSentClock = 0;
	lockNextFrame = false;
	PTAMInitializedClock = 0;
	lastPTAMMessage = "";

	flushMapKeypoints = false;

	node->publishCommand("u l PTAM has been reset.");
}

void PTAMWrapper::setPTAMPars(double minKFTimeDist, double minKFWiggleDist, double minKFDist)
{
	if(mpMapMaker != 0)
		mpMapMaker->minKFDist = minKFDist;
	if(mpMapMaker != 0)
		mpMapMaker->minKFWiggleDist = minKFWiggleDist;
	if(mpTracker != 0)
		mpTracker->minKFTimeDist = minKFTimeDist;

	this->minKFDist = minKFDist;
	this->minKFWiggleDist = minKFWiggleDist;
	this->minKFTimeDist = minKFTimeDist;
}

PTAMWrapper::~PTAMWrapper(void)
{
	if(mpCamera != 0) delete mpCamera;
	if(mpMap != 0) delete mpMap;
	if(mpMapMaker != 0) delete mpMapMaker;
	if(mpTracker != 0) delete mpTracker;
	if(predConvert != 0) delete predConvert;
	if(predIMUOnlyForScale != 0) delete predIMUOnlyForScale;
	if(imuOnlyPred != 0) delete imuOnlyPred;

}


void PTAMWrapper::startSystem()
{
	keepRunning = true;
	changeSizeNextRender = false;
	start();
}

void PTAMWrapper::stopSystem()
{
	keepRunning = false;
	new_frame_signal.notify_all();
	join();
}


void PTAMWrapper::run()
{
	std::cout << "Waiting for Video" << std::endl;

	// wait for firsst image
	while(!newImageAvailable)
		usleep(100000);	// sleep 100ms
	newImageAvailable = false;
	while(!newImageAvailable)
		usleep(100000);	// sleep 100ms

	// read image height and width
	frameWidth = mimFrameBW.size().x;
	frameHeight = mimFrameBW.size().y;

	ResetInternal();


	snprintf(charBuf,200,"Video resolution: %d x %d",frameWidth,frameHeight);
	ROS_INFO(charBuf);
	node->publishCommand(std::string("u l ")+charBuf);

	// create window
    myGLWindow = new GLWindow2(CVD::ImageRef(frameWidth,frameHeight), "PTAM Drone Camera Feed", this);
	myGLWindow->set_title("PTAM Drone Camera Feed");

	changeSizeNextRender = true;
	if(frameWidth < 640)
		desiredWindowSize = CVD::ImageRef(frameWidth*2,frameHeight*2);
	else
		desiredWindowSize = CVD::ImageRef(frameWidth,frameHeight);


	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	while(keepRunning)
	{
		if(newImageAvailable)
		{
			newImageAvailable = false;

			// copy to working copy
			mimFrameBW_workingCopy.copy_from(mimFrameBW);
			mimFrameTime_workingCopy = mimFrameTime;
			mimFrameSEQ_workingCopy = mimFrameSEQ;
			mimFrameTimeRos_workingCopy = mimFrameTimeRos;

			// release lock and do the work-intensive stuff.....
			lock.unlock();

			HandleFrame();


			if(changeSizeNextRender)
			{
				myGLWindow->set_size(desiredWindowSize);
				changeSizeNextRender = false;
			}

			// get lock again
			lock.lock();
		}
		else
			new_frame_signal.wait(lock);
	}

	lock.unlock();
	delete myGLWindow;
}

// called every time a new frame is available.
// needs to be able to 
// - (finally) roll forward filter
// - query it's state 
// - add a PTAM observation to filter.
void PTAMWrapper::HandleFrame()
{
	//printf("tracking Frame at ms=%d (from %d)\n",getMS(ros::Time::now()),mimFrameTime-filter->delayVideo);


	// prep data
	msg = "";
	ros::Time startedFunc = ros::Time::now();

	// reset?
	if(resetPTAMRequested)
		ResetInternal();


	// make filter thread-safe.
	// --------------------------- ROLL FORWARD TIL FRAME. This is ONLY done here. ---------------------------
	pthread_mutex_lock( &filter->filter_CS );
	//filter->predictUpTo(mimFrameTime,true, true);
	TooN::Vector<10> filterPosePrePTAM = filter->getPoseAtAsVec(mimFrameTime_workingCopy-filter->delayVideo,true);
	pthread_mutex_unlock( &filter->filter_CS );

	// ------------------------ do PTAM -------------------------
	myGLWindow->SetupViewport();
	myGLWindow->SetupVideoOrtho();
	myGLWindow->SetupVideoRasterPosAndZoom();



	// 1. transform with filter
	TooN::Vector<6> PTAMPoseGuess = filter->backTransformPTAMObservation(filterPosePrePTAM.slice<0,6>());
	// 2. convert to se3
	predConvert->setPosRPY(PTAMPoseGuess[0], PTAMPoseGuess[1], PTAMPoseGuess[2], PTAMPoseGuess[3], PTAMPoseGuess[4], PTAMPoseGuess[5]);
	// 3. multiply with rotation matrix	
	TooN::SE3<> PTAMPoseGuessSE3 = predConvert->droneToFrontNT * predConvert->globaltoDrone;


	// set
	mpTracker->setPredictedCamFromW(PTAMPoseGuessSE3);
	//mpTracker->setLastFrameLost((isGoodCount < -10), (videoFrameID%2 != 0));
	mpTracker->setLastFrameLost((isGoodCount < -20), (mimFrameSEQ_workingCopy%3 == 0));

	// track
	ros::Time startedPTAM = ros::Time::now();
	mpTracker->TrackFrame(mimFrameBW_workingCopy, true);
	TooN::SE3<> PTAMResultSE3 = mpTracker->GetCurrentPose();
	lastPTAMMessage = msg = mpTracker->GetMessageForUser();
	ros::Duration timePTAM= ros::Time::now() - startedPTAM;

	TooN::Vector<6> PTAMResultSE3TwistOrg = PTAMResultSE3.ln();

	node->publishTf(mpTracker->GetCurrentPose(),mimFrameTimeRos_workingCopy, mimFrameSEQ_workingCopy,"cam_front");


	// 1. multiply from left by frontToDroneNT.
	// 2. convert to xyz,rpy
	predConvert->setPosSE3_globalToDrone(predConvert->frontToDroneNT * PTAMResultSE3);
	TooN::Vector<6> PTAMResult = TooN::makeVector(predConvert->x, predConvert->y, predConvert->z, predConvert->roll, predConvert->pitch, predConvert->yaw);

	// 3. transform with filter
	TooN::Vector<6> PTAMResultTransformed = filter->transformPTAMObservation(PTAMResult);




	// init failed?
	if(mpTracker->lastStepResult == mpTracker->I_FAILED)
	{
		ROS_INFO("initializing PTAM failed, resetting!");
		resetPTAMRequested = true;
	}
	if(mpTracker->lastStepResult == mpTracker->I_SECOND)
	{
		PTAMInitializedClock = getMS();
		filter->setCurrentScales(TooN::makeVector(mpMapMaker->initialScaleFactor*1.2,mpMapMaker->initialScaleFactor*1.2,mpMapMaker->initialScaleFactor*1.2));
		mpMapMaker->currentScaleFactor = filter->getCurrentScales()[0];
		ROS_INFO("PTAM initialized!");
		ROS_INFO("initial scale: %f\n",mpMapMaker->initialScaleFactor*1.2);
		node->publishCommand("u l PTAM initialized (took second KF)");
		framesIncludedForScaleXYZ = -1;
		lockNextFrame = true;
		imuOnlyPred->resetPos();
	}
	if(mpTracker->lastStepResult == mpTracker->I_FIRST)
	{
		node->publishCommand("u l PTAM initialization started (took first KF)");
	}






	// --------------------------- assess result ------------------------------
	bool isGood = true;
	bool isVeryGood = true;
	// calculate absolute differences.
	TooN::Vector<6> diffs = PTAMResultTransformed - filterPosePrePTAM.slice<0,6>();
	for(int i=0;1<1;i++) diffs[i] = abs(diffs[i]);


	if(filter->getNumGoodPTAMObservations() < 10 && mpMap->IsGood())
	{
		isGood = true;
		isVeryGood = false;
	}
	else if(mpTracker->lastStepResult == mpTracker->I_FIRST ||
		mpTracker->lastStepResult == mpTracker->I_SECOND || 
		mpTracker->lastStepResult == mpTracker->I_FAILED ||
		mpTracker->lastStepResult == mpTracker->T_LOST ||
		mpTracker->lastStepResult == mpTracker->NOT_TRACKING ||
		mpTracker->lastStepResult == mpTracker->INITIALIZING)
		isGood = isVeryGood = false;
	else
	{
		// some chewy heuristic when to add and when not to.
		bool dodgy = mpTracker->lastStepResult == mpTracker->T_DODGY ||
			mpTracker->lastStepResult == mpTracker->T_RECOVERED_DODGY;

		// if yaw difference too big: something certainly is wrong.
		// maximum difference is 5 + 2*(number of seconds since PTAM observation).
		double maxYawDiff = 10.0 + (getMS()-lastGoodYawClock)*0.002;
		if(maxYawDiff > 20) maxYawDiff = 1000;
		if(false && diffs[5] > maxYawDiff) 
			isGood = false;

		if(diffs[5] < 10) 
			lastGoodYawClock = getMS();

		if(diffs[5] > 4.0) 
			isVeryGood = false;

		// if rp difference too big: something certainly is wrong.
		if(diffs[3] > 20 || diffs[4] > 20)
			isGood = false;

		if(diffs[3] > 3 || diffs[4] > 3 || dodgy)
			isVeryGood = false;
	}

	if(isGood)
	{
		if(isGoodCount < 0) isGoodCount = 0;
		isGoodCount++;
	}
	else
	{
		if(isGoodCount > 0) isGoodCount = 0;
		isGoodCount--;
		
		if(mpTracker->lastStepResult == mpTracker->T_RECOVERED_DODGY)
			isGoodCount = std::max(isGoodCount,-2);
		if(mpTracker->lastStepResult == mpTracker->T_RECOVERED_GOOD)
			isGoodCount = std::max(isGoodCount,-5);

	}

	TooN::Vector<10> filterPosePostPTAM;
	// --------------------------- scale estimation & update filter (REDONE) -----------------------------
	// interval length is always between 1s and 2s, to enshure approx. same variances.
	// if interval contained height inconsistency, z-distances are simply both set to zero, which does not generate a bias.
	// otherwise distances are scaled such that height is weighted more.
	// if isGood>=3 && framesIncludedForScale < 0			===> START INTERVAL
	// if 18 <= framesIncludedForScale <= 36 AND isGood>=3	===> ADD INTERVAL, START INTERVAL
	// if framesIncludedForScale > 36						===> set framesIncludedForScale=-1 

	// include!

	// TODO: make shure filter is handled properly with permanent roll-forwards.
	pthread_mutex_lock( &filter->filter_CS );
	if(filter->usePTAM && isGoodCount >= 3)
	{
		filter->addPTAMObservation(PTAMResult,mimFrameTime_workingCopy-filter->delayVideo);
	}
	else
		filter->addFakePTAMObservation(mimFrameTime_workingCopy-filter->delayVideo);

	filterPosePostPTAM = filter->getCurrentPoseSpeedAsVec();
	pthread_mutex_unlock( &filter->filter_CS );

	TooN::Vector<6> filterPosePostPTAMBackTransformed = filter->backTransformPTAMObservation(filterPosePostPTAM.slice<0,6>());


	// if interval is started: add one step.
	int includedTime = mimFrameTime_workingCopy - ptamPositionForScaleTakenTimestamp;
	if(framesIncludedForScaleXYZ >= 0) framesIncludedForScaleXYZ++;

	// if interval is overdue: reset & dont add
	if(includedTime > 3000) 
	{
		framesIncludedForScaleXYZ = -1;
	}

	if(isGoodCount >= 3)
	{
		// filter stuff
		lastScaleEKFtimestamp = mimFrameTime_workingCopy;

		if(includedTime >= 2000 && framesIncludedForScaleXYZ > 1)	// ADD! (if too many, was resetted before...)
		{
			TooN::Vector<3> diffPTAM = filterPosePostPTAMBackTransformed.slice<0,3>() - PTAMPositionForScale;
			bool zCorrupted, allCorrupted;
			float pressureStart = 0, pressureEnd = 0;
			TooN::Vector<3> diffIMU = evalNavQue(ptamPositionForScaleTakenTimestamp - filter->delayVideo + filter->delayXYZ,mimFrameTime_workingCopy - filter->delayVideo + filter->delayXYZ,&zCorrupted, &allCorrupted, &pressureStart, &pressureEnd);

			pthread_mutex_lock(&logScalePairs_CS);
			if(logfileScalePairs != 0)
				(*logfileScalePairs) <<
						pressureStart << " " <<
						pressureEnd << " " <<
						diffIMU[2] << " " <<
						diffPTAM[2] << std::endl;
			pthread_mutex_unlock(&logScalePairs_CS);


			if(!allCorrupted)
			{
				// filtering: z more weight, but only if not corrupted.
				double xyFactor = 0.05;
				double zFactor = zCorrupted ? 0 : 3;
			
				diffPTAM.slice<0,2>() *= xyFactor; diffPTAM[2] *= zFactor;
				diffIMU.slice<0,2>() *= xyFactor; diffIMU[2] *= zFactor;

				filter->updateScaleXYZ(diffPTAM, diffIMU, PTAMResult.slice<0,3>());
				mpMapMaker->currentScaleFactor = filter->getCurrentScales()[0];
			}
			framesIncludedForScaleXYZ = -1;	// causing reset afterwards
		}

		if(framesIncludedForScaleXYZ == -1)	// RESET!
		{
			framesIncludedForScaleXYZ = 0;
			PTAMPositionForScale = filterPosePostPTAMBackTransformed.slice<0,3>();
			//predIMUOnlyForScale->resetPos();	// also resetting z corrupted flag etc. (NOT REquired as reset is done in eval)
			ptamPositionForScaleTakenTimestamp = mimFrameTime_workingCopy;
		}
	}
	

	if(lockNextFrame && isGood)
	{
		filter->scalingFixpoint = PTAMResult.slice<0,3>();
		lockNextFrame = false;	
		//filter->useScalingFixpoint = true;

		snprintf(charBuf,500,"locking scale fixpoint to %.3f %.3f %.3f",PTAMResultTransformed[0], PTAMResultTransformed[1], PTAMResultTransformed[2]);
		ROS_INFO(charBuf);
		node->publishCommand(std::string("u l ")+charBuf);
	}


	// ----------------------------- Take KF? -----------------------------------
	if(!mapLocked && isVeryGood && (forceKF || mpMap->vpKeyFrames.size() < maxKF || maxKF <= 1))
	{
		mpTracker->TakeKF(forceKF);
		forceKF = false;
	}

	// ---------------- save PTAM status for KI --------------------------------
	if(mpTracker->lastStepResult == mpTracker->NOT_TRACKING)
		PTAMStatus = PTAM_IDLE;
	else if(mpTracker->lastStepResult == mpTracker->I_FIRST ||
		mpTracker->lastStepResult == mpTracker->I_SECOND ||
		mpTracker->lastStepResult == mpTracker->T_TOOK_KF)
		PTAMStatus = PTAM_TOOKKF;
	else if(mpTracker->lastStepResult == mpTracker->INITIALIZING)
		PTAMStatus = PTAM_INITIALIZING;
	else if(isVeryGood)
		PTAMStatus = PTAM_BEST;
	else if(isGood)
		PTAMStatus = PTAM_GOOD;
	else if(mpTracker->lastStepResult == mpTracker->T_DODGY ||
		mpTracker->lastStepResult == mpTracker->T_GOOD)
		PTAMStatus = PTAM_FALSEPOSITIVE;
	else
		PTAMStatus = PTAM_LOST;

	 
	// ----------------------------- update shallow map --------------------------
	if(!mapLocked && rand()%5==0)
	{
		pthread_mutex_lock(&shallowMapCS);
		mapPointsTransformed.clear();
		keyFramesTransformed.clear();
		for(unsigned int i=0;i<mpMap->vpKeyFrames.size();i++)
		{
			predConvert->setPosSE3_globalToDrone(predConvert->frontToDroneNT * mpMap->vpKeyFrames[i]->se3CfromW);
			TooN::Vector<6> CamPos = TooN::makeVector(predConvert->x, predConvert->y, predConvert->z, predConvert->roll, predConvert->pitch, predConvert->yaw);
			CamPos = filter->transformPTAMObservation(CamPos);
			predConvert->setPosRPY(CamPos[0], CamPos[1], CamPos[2], CamPos[3], CamPos[4], CamPos[5]);
			keyFramesTransformed.push_back(predConvert->droneToGlobal);
		}
		TooN::Vector<3> PTAMScales = filter->getCurrentScales();
		TooN::Vector<3> PTAMOffsets = filter->getCurrentOffsets().slice<0,3>();
		for(unsigned int i=0;i<mpMap->vpPoints.size();i++)
		{
			TooN::Vector<3> pos = (mpMap->vpPoints)[i]->v3WorldPos;
			pos[0] *= PTAMScales[0];
			pos[1] *= PTAMScales[1];
			pos[2] *= PTAMScales[2];
			pos += PTAMOffsets;
			mapPointsTransformed.push_back(pos);
		}

		// flush map keypoints
		if(flushMapKeypoints)
		{
			std::ofstream* fle = new std::ofstream();
			fle->open("pointcloud.txt");

			for(unsigned int i=0;i<mapPointsTransformed.size();i++)
			{
				(*fle) << mapPointsTransformed[i][0] << " "
					   << mapPointsTransformed[i][1] << " "
					   << mapPointsTransformed[i][2] << std::endl;
			}

			fle->flush();
			fle->close();

			printf("FLUSHED %d KEYPOINTS to file pointcloud.txt\n\n",mapPointsTransformed.size());

			flushMapKeypoints = false;
		}


		pthread_mutex_unlock(&shallowMapCS);

	}



	// ---------------------- output and render! ---------------------------
	ros::Duration timeALL = ros::Time::now() - startedFunc;
	if(isVeryGood) snprintf(charBuf,1000,"\nQuality: best            ");
	else if(isGood) snprintf(charBuf,1000,"\nQuality: good           ");
	else snprintf(charBuf,1000,"\nQuality: lost                       ");
	
	snprintf(charBuf+20,800, "scale: %.3f (acc: %.3f)                            ",filter->getCurrentScales()[0],(double)filter->getScaleAccuracy());
	snprintf(charBuf+50,800, "PTAM time: %i ms                            ",(int)(1000*timeALL.toSec()));
	snprintf(charBuf+68,800, "(%i ms total)  ",(int)(1000*timeALL.toSec()));
	if(mapLocked) snprintf(charBuf+83,800, "m.l. ");
	else snprintf(charBuf+83,800, "     ");
	if(filter->allSyncLocked) snprintf(charBuf+88,800, "s.l. ");
	else snprintf(charBuf+88,800, "     ");


	msg += charBuf;

	if(mpMap->IsGood())
	{
		if(drawUI == UI_DEBUG)
		{
			snprintf(charBuf,1000,"\nPTAM Diffs:              ");
			snprintf(charBuf+13,800, "x: %.3f                          ",diffs[0]);
			snprintf(charBuf+23,800, "y: %.3f                          ",diffs[1]);
			snprintf(charBuf+33,800, "z: %.3f                          ",diffs[2]);
			snprintf(charBuf+43,800, "r: %.2f                          ",diffs[3]);
			snprintf(charBuf+53,800, "p: %.2f                          ",diffs[4]);
			snprintf(charBuf+63,800, "y: %.2f",diffs[5]);
			msg += charBuf;


			snprintf(charBuf,1000,"\nPTAM Pose:              ");
			snprintf(charBuf+13,800, "x: %.3f                          ",PTAMResultTransformed[0]);
			snprintf(charBuf+23,800, "y: %.3f                          ",PTAMResultTransformed[1]);
			snprintf(charBuf+33,800, "z: %.3f                          ",PTAMResultTransformed[2]);
			snprintf(charBuf+43,800, "r: %.2f                          ",PTAMResultTransformed[3]);
			snprintf(charBuf+53,800, "p: %.2f                          ",PTAMResultTransformed[4]);
			snprintf(charBuf+63,800, "y: %.2f",PTAMResultTransformed[5]);
			msg += charBuf;


			snprintf(charBuf,1000,"\nPTAM WiggleDist:              ");
			snprintf(charBuf+18,800, "%.3f                          ",mpMapMaker->lastWiggleDist);
			snprintf(charBuf+24,800, "MetricDist: %.3f",mpMapMaker->lastMetricDist);
			msg += charBuf;
		}
	}

	if(drawUI != UI_NONE)
	{
		// render grid
		predConvert->setPosRPY(filterPosePostPTAM[0], filterPosePostPTAM[1], filterPosePostPTAM[2], filterPosePostPTAM[3], filterPosePostPTAM[4], filterPosePostPTAM[5]);

		//renderGrid(predConvert->droneToFrontNT * predConvert->globaltoDrone);
		//renderGrid(PTAMResultSE3);


		// draw HUD
		//if(mod->getControlSystem()->isControlling())
		{
			myGLWindow->SetupViewport();
			myGLWindow->SetupVideoOrtho();
			myGLWindow->SetupVideoRasterPosAndZoom();

			//glDisable(GL_LINE_SMOOTH);
			glLineWidth(2);
			glBegin(GL_LINES);
			glColor3f(0,0,1);

			glVertex2f(0,frameHeight/2);
			glVertex2f(frameWidth,frameHeight/2);

			glVertex2f(frameWidth/2,0);
			glVertex2f(frameWidth/2,frameHeight);

			// 1m lines
			glVertex2f(0.25*frameWidth,0.48*frameHeight);
			glVertex2f(0.25*frameWidth,0.52*frameHeight);
			glVertex2f(0.75*frameWidth,0.48*frameHeight);
			glVertex2f(0.75*frameWidth,0.52*frameHeight);
			glVertex2f(0.48*frameWidth,0.25*frameHeight);
			glVertex2f(0.52*frameWidth,0.25*frameHeight);
			glVertex2f(0.48*frameWidth,0.75*frameHeight);
			glVertex2f(0.52*frameWidth,0.75*frameHeight);

			glEnd();
		}


		myGLWindow->DrawCaption(msg);
	}

	lastPTAMResultRaw = PTAMResultSE3; 
	// ------------------------ LOG --------------------------------------
	// log!
	if(node->logfilePTAM != NULL)
	{
		TooN::Vector<3> scales = filter->getCurrentScalesForLog();
		TooN::Vector<3> sums = TooN::makeVector(0,0,0);
		TooN::Vector<6> offsets = filter->getCurrentOffsets();
		pthread_mutex_lock(&(node->logPTAM_CS));
		// log:
		// - filterPosePrePTAM estimated for videoFrameTimestamp-delayVideo.
		// - PTAMResulttransformed estimated for videoFrameTimestamp-delayVideo. (using imu only for last step)
		// - predictedPoseSpeed estimated for lastNfoTimestamp+filter->delayControl	(actually predicting)
		// - predictedPoseSpeedATLASTNFO estimated for lastNfoTimestamp	(using imu only)
		if(node->logfilePTAM != NULL)
			(*(node->logfilePTAM)) << (isGood ? (isVeryGood ? 2 : 1) : 0) << " " <<
				(mimFrameTime_workingCopy-filter->delayVideo) << " " << filterPosePrePTAM[0] << " " << filterPosePrePTAM[1] << " " << filterPosePrePTAM[2] << " " << filterPosePrePTAM[3] << " " << filterPosePrePTAM[4] << " " << filterPosePrePTAM[5] << " " << filterPosePrePTAM[6] << " " << filterPosePrePTAM[7] << " " << filterPosePrePTAM[8] << " " << filterPosePrePTAM[9] << " " <<
				filterPosePostPTAM[0] << " " << filterPosePostPTAM[1] << " " << filterPosePostPTAM[2] << " " << filterPosePostPTAM[3] << " " << filterPosePostPTAM[4] << " " << filterPosePostPTAM[5] << " " << filterPosePostPTAM[6] << " " << filterPosePostPTAM[7] << " " << filterPosePostPTAM[8] << " " << filterPosePostPTAM[9] << " " << 
				PTAMResultTransformed[0] << " " << PTAMResultTransformed[1] << " " << PTAMResultTransformed[2] << " " << PTAMResultTransformed[3] << " " << PTAMResultTransformed[4] << " " << PTAMResultTransformed[5] << " " << 
				scales[0] << " " << scales[1] << " " << scales[2] << " " << 
				offsets[0] << " " << offsets[1] << " " << offsets[2] << " " << offsets[3] << " " << offsets[4] << " " << offsets[5] << " " <<
				sums[0] << " " << sums[1] << " " << sums[2] << " " << 
				PTAMResult[0] << " " << PTAMResult[1] << " " << PTAMResult[2] << " " << PTAMResult[3] << " " << PTAMResult[4] << " " << PTAMResult[5] << " " <<
				PTAMResultSE3TwistOrg[0] << " " << PTAMResultSE3TwistOrg[1] << " " << PTAMResultSE3TwistOrg[2] << " " << PTAMResultSE3TwistOrg[3] << " " << PTAMResultSE3TwistOrg[4] << " " << PTAMResultSE3TwistOrg[5] << " " <<
				videoFramePing << " " << mimFrameTimeRos_workingCopy << " " << mimFrameSEQ_workingCopy << std::endl;

		pthread_mutex_unlock(&(node->logPTAM_CS));
	}

	myGLWindow->swap_buffers();
	myGLWindow->HandlePendingEvents();

}


// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void PTAMWrapper::renderGrid(TooN::SE3<> camFromWorld)
{
	myGLWindow->SetupViewport();
	myGLWindow->SetupVideoOrtho();
	myGLWindow->SetupVideoRasterPosAndZoom();

	camFromWorld.get_translation() *= 1;

	// The colour of the ref grid shows if the coarse stage of tracking was used
	// (it's turned off when the camera is sitting still to reduce jitter.)
	glColor4f(0,0,0,0.6);
  
	// The grid is projected manually, i.e. GL receives projected 2D coords to draw.
	int nHalfCells = 5;
	int nTot = nHalfCells * 2 + 1;
	CVD::Image<Vector<2> >  imVertices(CVD::ImageRef(nTot,nTot));
	for(int i=0; i<nTot; i++)
		for(int j=0; j<nTot; j++)
		{
			Vector<3> v3;
			v3[0] = (i - nHalfCells) * 1;
			v3[1] = (j - nHalfCells) * 1;
			v3[2] = 0.0;
			Vector<3> v3Cam = camFromWorld * v3;
			//v3Cam[2] *= 100;
			if(v3Cam[2] < 0.001)
				v3Cam = TooN::makeVector(100000*v3Cam[0],100000*v3Cam[1],0.0001);

			imVertices[i][j] = mpCamera->Project(TooN::project(v3Cam))*0.5;
		}

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(2);
	for(int i=0; i<nTot; i++)
	{
		glBegin(GL_LINE_STRIP);
		for(int j=0; j<nTot; j++)
		CVD::glVertex(imVertices[i][j]);
		glEnd();
      
		glBegin(GL_LINE_STRIP);
		for(int j=0; j<nTot; j++)
		CVD::glVertex(imVertices[j][i]);
		glEnd();
	};
  
  glLineWidth(1);
  glColor3f(1,0,0);



}

TooN::Vector<3> PTAMWrapper::evalNavQue(unsigned int from, unsigned int to, bool* zCorrupted, bool* allCorrupted, float* out_start_pressure, float* out_end_pressure)
{
	predIMUOnlyForScale->resetPos();

	int firstAdded = 0, lastAdded = 0;
	pthread_mutex_lock(&navInfoQueueCS);
	int skipped=0;
	int used = 0;
	int firstZ = 0;

	float sum_first=0, num_first=0, sum_last=0, num_last=0;
	int pressureAverageRange = 100;


	for(std::deque<ardrone_autonomy::Navdata>::iterator cur = navInfoQueue.begin();
			cur != navInfoQueue.end();
			)
	{
		int curStampMs = getMS(cur->header.stamp);

		if(curStampMs < (int)from-pressureAverageRange)
			cur = navInfoQueue.erase(cur);
		else
		{
			if(curStampMs >= (int)from-pressureAverageRange && curStampMs <= (int)from+pressureAverageRange)
			{
				sum_first += cur->pressure;
				num_first++;
			}

			if(curStampMs >= (int)to-pressureAverageRange && curStampMs <= (int)to+pressureAverageRange)
			{
				sum_last += cur->pressure;
				num_last++;
			}
			cur++;
		}
	}

	for(std::deque<ardrone_autonomy::Navdata>::iterator cur = navInfoQueue.begin();
			cur != navInfoQueue.end();
			cur++
			)
	{
		int frontStamp = getMS(cur->header.stamp);
		if(frontStamp < from)		// packages before: delete
		{
			//navInfoQueue.pop_front();
			skipped++;
		}
		else if(frontStamp >= from && frontStamp <= to)
		{
			if(firstAdded == 0) 
			{
				firstAdded = frontStamp;
				firstZ = cur->altd;
				predIMUOnlyForScale->z = firstZ*0.001;	// avoid height check initially!
			}
			lastAdded = frontStamp;
			// add
			predIMUOnlyForScale->predictOneStep(&(*cur));
			// pop
			//navInfoQueue.pop_front();
			used++;
		}
		else
			break;

	}
	//printf("QueEval: before: %i; skipped: %i, used: %i, left: %i\n", totSize, skipped, used, navInfoQueue.size());
	predIMUOnlyForScale->z -= firstZ*0.001;	// make height to height-diff

	*zCorrupted = predIMUOnlyForScale->zCorrupted;
	*allCorrupted = abs(firstAdded - (int)from) + abs(lastAdded - (int)to) > 80;
	pthread_mutex_unlock(&navInfoQueueCS);

	if(*allCorrupted)
		printf("scalePackage corrupted (imu data gap for %ims)\n",abs(firstAdded - (int)from) + abs(lastAdded - (int)to));
	else if(*zCorrupted)
		printf("scalePackage z corrupted (jump in meters: %.3f)!\n",predIMUOnlyForScale->zCorruptedJump);

	printf("first: %f (%f); last: %f (%f)=> diff: %f (z alt diff: %f)\n",
			sum_first/num_first,
			num_first,
			sum_last/num_last,
			num_last,
			sum_last/num_last - sum_first/num_first,
			predIMUOnlyForScale->z
	);


	*out_end_pressure = sum_last/num_last;
	*out_start_pressure = sum_first/num_first;

	return TooN::makeVector(predIMUOnlyForScale->x,predIMUOnlyForScale->y,predIMUOnlyForScale->z);
}

void PTAMWrapper::newNavdata(ardrone_autonomy::Navdata* nav)
{
	lastNavinfoReceived = *nav;

	if(getMS(lastNavinfoReceived.header.stamp) > 2000000)
	{
		printf("PTAMSystem: ignoring navdata package with timestamp %f\n", lastNavinfoReceived.tm);
		return;
	}
	if(lastNavinfoReceived.header.seq > 2000000 || lastNavinfoReceived.header.seq < 0)
	{
		printf("PTAMSystem: ignoring navdata package with ID %i\n", lastNavinfoReceived.header.seq);
		return;
	}

	// correct yaw with filter-yaw (!):
	lastNavinfoReceived.rotZ = filter->getCurrentPose()[5];

	pthread_mutex_lock( &navInfoQueueCS );
	navInfoQueue.push_back(lastNavinfoReceived);

	if(navInfoQueue.size() > 1000)	// respective 5s
	{
		navInfoQueue.pop_front();
		if(!navQueueOverflown)
			printf("NavQue Overflow detected!\n");
		navQueueOverflown = true;
	}
	pthread_mutex_unlock( &navInfoQueueCS );

	//filter->setPing(nav->pingNav, nav->pingVid);

	imuOnlyPred->yaw = filter->getCurrentPose()[5];
	imuOnlyPred->predictOneStep(&lastNavinfoReceived);
}

void PTAMWrapper::newImage(sensor_msgs::ImageConstPtr img)
{

	// convert to CVImage
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);


	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	// copy to internal image, convert to bw, set flag.
	if(ros::Time::now() - img->header.stamp > ros::Duration(30.0))
		mimFrameTimeRos = (ros::Time::now()-ros::Duration(0.001));
	else
		mimFrameTimeRos = (img->header.stamp);

	mimFrameTime = getMS(mimFrameTimeRos);

	//mimFrameTime = getMS(img->header.stamp);
	mimFrameSEQ = img->header.seq;

	// copy to mimFrame.
	// TODO: make this threadsafe (save pointer only and copy in HandleFrame)
	if(mimFrameBW.size().x != img->width || mimFrameBW.size().y != img->height)
		mimFrameBW.resize(CVD::ImageRef(img->width, img->height));

	memcpy(mimFrameBW.data(),cv_ptr->image.data,img->width * img->height);
	newImageAvailable = true;

	lock.unlock();
	new_frame_signal.notify_all();
}



void PTAMWrapper::on_key_down(int key)
{
	if(key == 114) // r
	{
		node->publishCommand("p reset");
	}
	if(key == 117) // u
	{
		node->publishCommand("p toggleUI");
	}
	if(key == 32) // Space
	{
		node->publishCommand("p space");
	}
	if(key == 107) // k
	{
		node->publishCommand("p keyframe");
	}
	if(key == 108) // l
	{
		node->publishCommand("toggleLog");
	}
	if(key == 115) // s
	{
		pthread_mutex_lock(&logScalePairs_CS);
		if(logfileScalePairs == 0)
		{
			logfileScalePairs = new std::ofstream();
			logfileScalePairs->open ("logScalePairs.txt");
			printf("\nSTART logging scale pairs\n\n");
		}
		else
		{
			logfileScalePairs->flush();
			logfileScalePairs->close();
			delete logfileScalePairs;
			logfileScalePairs = NULL;
			printf("\nEND logging scale pairs\n\n");
		}
		pthread_mutex_unlock(&logScalePairs_CS);
	}

	if(key == 109) // m
	{
		node->publishCommand("p toggleLockMap");
	}

	if(key == 110) // n
	{

		node->publishCommand("p toggleLockSync");
	}

	if(key == 116) // t
	{

		flushMapKeypoints = true;
	}

}


// reached by typing "df p COMMAND" into console
bool PTAMWrapper::handleCommand(std::string s)
{
	if(s.length() == 5 && s.substr(0,5) == "space")
	{
  		mpTracker->pressSpacebar();
	}

	// ptam reset: resets only PTAM, keeps filter state.
	if(s.length() == 5 && s.substr(0,5) == "reset")
	{
		//filter->clearPTAM();
  		Reset();

	}

	if(s.length() == 8 && s.substr(0,8) == "keyframe")
	{
		forceKF = true;
	}

	if(s.length() == 8 && s.substr(0,8) == "toggleUI")
	{
		if(drawUI == UI_NONE) drawUI = UI_DEBUG;
		else if(drawUI == UI_DEBUG) drawUI = UI_PRES;
		else if(drawUI == UI_PRES) drawUI = UI_NONE;
		else drawUI = UI_PRES;
	}

	if(s.length() == 11 && s.substr(0,11) == "lockScaleFP")
	{
		lockNextFrame = true;
	}

	if(s.length() == 13 && s.substr(0,13) == "toggleLockMap")
	{
		mapLocked = !mapLocked;


		if(mapLocked)
		{
			node->publishCommand("u l PTAM map locked.");
			printf("\n\nMAP LOCKED!\n\n\n");
		}
		else
		{
			printf("\n\nMAP UNLOCKED!\n\n\n");
			node->publishCommand("u l PTAM map UNlocked.");
		}
	}

	if(s.length() == 14 && s.substr(0,14) == "toggleLockSync")
	{
		filter->allSyncLocked = !filter->allSyncLocked;


		if(filter->allSyncLocked)
		{
			printf("\n\nSYNC LOCKED!\n\n\n");
			node->publishCommand("u l PTAM sync locked.");
		}
		else
		{
			printf("\n\nSYNC UNLOCKED!\n\n\n");
			node->publishCommand("u l PTAM sync UNlocked.");
		}
	}

	return true;
}

void PTAMWrapper::on_mouse_down(CVD::ImageRef where, int state, int button)
{
	double x = 4*(where.x/(double)this->myGLWindow->size().x - 0.5);
	double y = -4*(where.y/(double)this->myGLWindow->size().y - 0.5);
	char bf[100];


	node->publishCommand("c clearCommands");
	node->publishCommand("c lockScaleFP");

	if(button == 1)
		snprintf(bf,100,"c moveByRel %.3f %.3f 0 0",x,y);
	else
		snprintf(bf,100,"c moveByRel 0 0 %.3f %.3f",y,x*45);

	node->publishCommand(bf);
}
