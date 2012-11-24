#pragma once
/*
 * PID controller for the AR.Drone.
 * 
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * 
 * and
 * 
 * Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the Workshop on Visual Control of Mobile Robots (ViCoMoR) at the IEEE/RJS International Conference on Intelligent Robot Systems (IROS), 2012. 
 *
 *
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
 

#include "TooN/se3.h"
#include <queue>
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"

class ControlNode;

struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};


struct DronePosition
{
public:
	double yaw;
	TooN::Vector<3> pos;
	inline DronePosition(TooN::Vector<3> pos, double yaw)
		: pos(pos), yaw(yaw) {}
	inline DronePosition(){ yaw=0; pos=TooN::makeVector(0,0,0);}
};

class DroneController
{
private:
	ControlCommand lastSentControl;
	
	// currentTarget.
	DronePosition target;
	bool targetValid;

	// used for integral term
	TooN::Vector<4> targetNew;	// 0=target has been reached before
								// 1=target is new

	// need to keep track of integral terms
	TooN::Vector<4> i_term;
	TooN::Vector<4> last_err;
	TooN::Vector<4> speedAverages;

	double lastTimeStamp;
	double targetSetAtClock;
	ControlCommand hoverCommand;



	// filled with info (on update)
	bool  ptamIsGood;
	double scaleAccuracy;
	void calcControl(TooN::Vector<4> new_err, TooN::Vector<4> d_error, double yaw);

public:

	// generates and sends a new control command to the drone, based on the currently active command ant the drone's position.
	ControlCommand update(tum_ardrone::filter_stateConstPtr);

	ControlNode* node;

	// for logging, gets filled with recent infos on control.
	TooN::Vector<28> logInfo;

	// adds a waypoint
	void setTarget(DronePosition newTarget);
	void clearTarget();
	DronePosition getCurrentTarget();
	ControlCommand getLastControl();

	// gets last error
	TooN::Vector<4> getLastErr();

	DroneController(void);
	~DroneController(void);





	// PID control parameters. settable via dynamic_reconfigure
	// target is where i want to get to.
	// pose and yaw are where i am.
	double max_yaw;
	double max_rp;
	double max_gaz_rise;
	double max_gaz_drop;

	double rise_fac;
	double agressiveness;

	double Ki_yaw;
	double Kd_yaw;
	double Kp_yaw;

	double Ki_gaz;
	double Kd_gaz;
	double Kp_gaz;

	double Ki_rp;
	double Kd_rp;
	double Kp_rp;

};

