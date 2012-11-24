/*
 * drone_stateestimation
 * This node publishes the drones estimated position, based on 
 * - drone nacdata
 * - PTAM
 * - sent control.
 *
 * This node does not send any control commands to the drone.
 *
 *
 * For further details see Camera-Based Navigation of a Low-Cost Quadrocopter (J. Engel, J. Sturm, D. Cremers)
 * In Proc. of the International Conference on Intelligent Robot Systems (IROS), 2012.
 * Author: Jakob Engel <jajuengel@gmail.com>
 *
 */
 
 
#include "EstimationNode.h"
#include "ros/ros.h"
#include "PTAMWrapper.h"
#include "MapView.h"


// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "drone_stateestimation");

  ROS_INFO("Started TUM ArDrone Stateestimation Node.");

  EstimationNode estimator;

  dynamic_reconfigure::Server<tum_ardrone::StateestimationParamsConfig> srv;
  dynamic_reconfigure::Server<tum_ardrone::StateestimationParamsConfig>::CallbackType f;
  f = boost::bind(&EstimationNode::dynConfCb, &estimator, _1, _2);
  srv.setCallback(f);

  estimator.ptamWrapper->startSystem();
  estimator.mapView->startSystem();

  estimator.Loop();

  estimator.mapView->stopSystem();
  estimator.ptamWrapper->stopSystem();

  return 0;
}
