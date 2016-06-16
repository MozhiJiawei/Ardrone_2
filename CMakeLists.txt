cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(ARDRONE_L-H)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
  roscpp
  tf
  rospy
)

find_package( OpenCV REQUIRED )
include_directories(/home/mozhi/catkin_ws/src/ardrone_autonomy/include)
include_directories(/usr/local/include/VisualSLAM)
#-DCMAKE_BUILD_TYPE=Debug

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
        src/main.cpp
        
        src/ROSThread.cpp
        src/IMURecorder.cpp
        src/IMUData.cpp
        src/VideoRecorder.cpp
        src/CMDReciever.cpp
        src/ArdroneTf.cpp
        src/RefPoseBr.cpp

        src/ARDrone.cpp
        src/PIDController.cpp

        src/GridDetector.cpp
        src/AffineTransform.cpp
        src/PredictNumber.cpp
        src/SearchNumber.cpp
				src/ImgRecon.cpp
#blobtrack_sample.cpp
#enteringblobdetection.cpp
        #src/ARVideo.cpp
        #src/DroneIMUThread.cpp
        #src/IMUVideoSync.cpp
)
set(MoSLAM_LIB
        libVisualSLAM.a
        GL
        GLU
        glut
        lapack
        blas
        m
)

## Declare a cpp executable
add_executable(ARDRONE_L-H ${SOURCE_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(ARDRONE_L-H
  ${catkin_LIBRARIES}
  ${MoSLAM_LIB}
  ${OpenCV_LIBS}
)
