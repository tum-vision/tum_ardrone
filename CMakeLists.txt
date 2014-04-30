#
# Author: Jakob Engel <jajuengel@gmail.com>
# Contributor: Stefan Wilkes <stefan.wilkes@gmail.com>
#
cmake_minimum_required(VERSION 2.8.3)
project(tum_ardrone)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  roscpp
  rospy
)

# Compile third party libs
include(ExternalProject)
ExternalProject_Add(thirdparty
	URL ${PROJECT_SOURCE_DIR}/thirdparty/thirdparty.tar.gz
	PREFIX ${CMAKE_BINARY_DIR}/thirdparty
	CONFIGURE_COMMAND ""
	BUILD_COMMAND make
	INSTALL_COMMAND ""
	BUILD_IN_SOURCE 1
)

# ------------------- add dynamic reconfigure api ------------------------------------
generate_dynamic_reconfigure_options(
  cfg/AutopilotParams.cfg
  cfg/GUIParams.cfg
  cfg/StateestimationParams.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES filter_state.msg)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	SetReference.srv
	SetMaxControl.srv
	SetInitialReachDistance.srv
	SetStayWithinDistance.srv
	SetStayTime.srv
)

## Generate added messages 
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS message_runtime std_msgs ardrone_autonomy)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# --------------------------- stateestimation & PTAM --------------------------------
# set header ans source files
set(STATEESTIMATION_SOURCE_FILES       
	src/stateestimation/GLWindow2.cc
	src/stateestimation/GLWindowMenu.cc  
	src/stateestimation/main_stateestimation.cpp
	src/stateestimation/DroneKalmanFilter.cpp
	src/stateestimation/Predictor.cpp
  src/stateestimation/PTAMWrapper.cpp
  src/stateestimation/MapView.cpp
  src/stateestimation/EstimationNode.cpp
  src/stateestimation/PTAM/ATANCamera.cc
  src/stateestimation/PTAM/Bundle.cc
  src/stateestimation/PTAM/HomographyInit.cc
  src/stateestimation/PTAM/KeyFrame.cc
  src/stateestimation/PTAM/Map.cc
  src/stateestimation/PTAM/MapMaker.cc
  src/stateestimation/PTAM/MapPoint.cc
  src/stateestimation/PTAM/MiniPatch.cc
  src/stateestimation/PTAM/PatchFinder.cc
  src/stateestimation/PTAM/Relocaliser.cc
  src/stateestimation/PTAM/ShiTomasi.cc
  src/stateestimation/PTAM/SmallBlurryImage.cc
  src/stateestimation/PTAM/Tracker.cc
)
set(STATEESTIMATION_HEADER_FILES    
  src/stateestimation/GLWindow2.h 
  src/stateestimation/GLWindowMenu.h    
  src/stateestimation/MouseKeyHandler.h  
  src/HelperFunctions.h   
  src/stateestimation/DroneKalmanFilter.h        
  src/stateestimation/Predictor.h 
  src/stateestimation/PTAMWrapper.h
  src/stateestimation/MapView.h
  src/stateestimation/EstimationNode.h
  src/stateestimation/PTAM/ATANCamera.h
  src/stateestimation/PTAM/Bundle.h
  src/stateestimation/PTAM/customFixes.h
  src/stateestimation/PTAM/HomographyInit.h
  src/stateestimation/PTAM/KeyFrame.h
  src/stateestimation/PTAM/LevelHelpers.h
  src/stateestimation/PTAM/Map.h
  src/stateestimation/PTAM/MapMaker.h
  src/stateestimation/PTAM/MapPoint.h
  src/stateestimation/PTAM/MEstimator.h
  src/stateestimation/PTAM/MiniPatch.h
  src/stateestimation/PTAM/OpenGL.h
  src/stateestimation/PTAM/PatchFinder.h
  src/stateestimation/PTAM/Relocaliser.h
  src/stateestimation/PTAM/settingsCustom.h
  src/stateestimation/PTAM/ShiTomasi.h
  src/stateestimation/PTAM/SmallBlurryImage.h
  src/stateestimation/PTAM/SmallMatrixOpts.h
  src/stateestimation/PTAM/TrackerData.h
  src/stateestimation/PTAM/Tracker.h
  src/stateestimation/PTAM/VideoSource.h
)

# set required libs and headers
include_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/TooN/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/include 
)  

link_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/lib 
)
set(PTAM_LIBRARIES GL glut cvd GVars3 blas lapack)
add_definitions(-DKF_REPROJ)

# build!
add_executable(drone_stateestimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})
set_target_properties(drone_stateestimation PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3") 
target_link_libraries(drone_stateestimation ${PTAM_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(drone_stateestimation thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ------------------------- autopilot & KI -----------------------------------------
# set header ans source files
set(AUTOPILOT_SOURCE_FILES         
	src/autopilot/main_autopilot.cpp  
	src/autopilot/ControlNode.cpp
	src/autopilot/DroneController.cpp
	src/autopilot/KI/KILand.cpp
	src/autopilot/KI/KIAutoInit.cpp
	src/autopilot/KI/KIFlyTo.cpp
)
set(AUTOPILOT_HEADER_FILES        
	src/autopilot/ControlNode.h
	src/autopilot/DroneController.h
	src/autopilot/KI/KILand.h
	src/autopilot/KI/KIAutoInit.h
	src/autopilot/KI/KIFlyTo.h
	src/autopilot/KI/KIProcedure.h
)

# build!
add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
target_link_libraries(drone_autopilot ${catkin_LIBRARIES})
add_dependencies(drone_autopilot thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ---------------------------- GUI --------------------------------------------------
# set header ans source files
set(GUI_SOURCE_FILES         
	src/UINode/main_GUI.cpp  
	src/UINode/tum_ardrone_gui.cpp  
	src/UINode/RosThread.cpp  
	src/UINode/PingThread.cpp  
)

set(GUI_HEADER_FILES        
	src/UINode/tum_ardrone_gui.h 
	src/UINode/RosThread.h  
	src/UINode/PingThread.h  
)

# *.ui
set(GUI_UI_FILES        
	src/UINode/tum_ardrone_gui.ui 
)

# *.qrc
set(GUI_RESOURCE_FILES        
)

# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# build!
add_executable(drone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(drone_gui ${QT_LIBRARIES} cvd ${catkin_LIBRARIES})
add_dependencies(drone_gui thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


