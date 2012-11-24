------------ camera calibration: -----------------------
- front, old drone: 0.672049, 0.899033, 0.502065, 0.513876, 0.271972
- front, new drone: 0.771557, 1.368560, 0.552779, 0.444056, 1.156010

calibrate with ethzasl_ptam.
to work with colored images, in src/CameraCalibrator.cc change:
- add #include <cv_bridge/cv_bridge.h>
- change function imageCallback(...) to
void CameraCalibrator::imageCallback(const sensor_msgs::ImageConstPtr & img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	if(mCurrentImage.size().x != img->width || mCurrentImage.size().y != img->height)
		mCurrentImage.resize(CVD::ImageRef(img->width, img->height));

	memcpy(mCurrentImage.data(),cv_ptr->image.data,img->width * img->height);

	mNewImage = true;
}

  

------------ Parameters: c1 to c8 -----------------------------
can be estimated easily by 
- recording a flight: rosbag record -O calibFlight.bag /ardrone/image_raw /ardrone/navdata /cmd_vel
- playing back that flight: rosbag play -l calibFlight.bag
- starting two stateestimation nodes, one with remapped name and output:
  - rosrun tum_ardrone drone_stateestimation __name:=drone_stateestimationn2 /ardrone/predictedPose:=/ardrone/predictedPose2
  - rosrun tum_ardrone drone_stateestimation
- plotting the respective estimated values
  - e.g.: rxplot /ardrone/predictedPose/dx,/ardrone/predictedPose2/dx
- using dynamic reconfigure to 
  - in drone_stateestimation2, use only control gains
  - in drone_stateestimation, use NO control gains, but instead navdata / speeds / PTAM.
- setting c_i in drone_stateestimation2 such that graphs match best (play around).




------------ Parameters: PID control -----------------------------
approximate in "simulation" based on c1 to c8:
- play back any record, to make stateestimation run (dont play /cmd_vel)
  - rosbag play -l calibFlightZ.bag --topics /ardrone/image_raw /ardrone/navdata
- run stateestimation, in dynamic reconfigure set only control updates to be used
- run gui and autopilot, and control
- plot control / respective pose e.g. with 
  - rxplot /ardrone/predictedPose/yaw /cmd_vel/angular/z



