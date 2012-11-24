#
# Author: Jakob Engel <jajuengel@gmail.com>
#
 
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# ------------------- add dynamic reconfigure api ------------------------------------
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()




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
  ${PROJECT_SOURCE_DIR}/thirdparty/TooN/include 
  ${PROJECT_SOURCE_DIR}/thirdparty/libcvd/include 
  ${PROJECT_SOURCE_DIR}/thirdparty/gvars3/include 
)  

link_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/libcvd/lib 
  ${PROJECT_SOURCE_DIR}/thirdparty/gvars3/lib 
)
set(PTAM_LIBRARIES GL glut cvd GVars3 blas lapack)
add_definitions(-DKF_REPROJ)

# build!
rosbuild_add_executable(drone_stateestimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})
rosbuild_add_compile_flags(drone_stateestimation -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3) 
target_link_libraries(drone_stateestimation ${PTAM_LIBRARIES})



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
rosbuild_add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})



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
rosbuild_add_executable(drone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(drone_gui ${QT_LIBRARIES} cvd)




# ---------------------------- Messages & Services --------------------------------------------------
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
