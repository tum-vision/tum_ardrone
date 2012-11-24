/*
 * drone_autopilot:
 * This node controls the drone. requires drone_stateestimation.
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
 
 
 
#include "ControlNode.h"
#include "ros/ros.h"
#include <ros/package.h>
#include "boost/thread.hpp"
#include <signal.h>

// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_autopilot");

  ROS_INFO("Started TUM ArDrone Autopilot Node.");

  ControlNode controlNode;

  dynamic_reconfigure::Server<tum_ardrone::AutopilotParamsConfig> srv;
  dynamic_reconfigure::Server<tum_ardrone::AutopilotParamsConfig>::CallbackType f;
  f = boost::bind(&ControlNode::dynConfCb, &controlNode, _1, _2);
  srv.setCallback(f);

  controlNode.Loop();

  return 0;
}
