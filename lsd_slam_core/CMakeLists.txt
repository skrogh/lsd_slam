cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE Release)
#
# rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)
find_package(Boost REQUIRED COMPONENTS thread system)

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^x86_64" )
	message(STATUS "Architecture is x84_64, compiling with SSE")
	# SSE flags
	add_definitions("-DENABLE_SSE")

	# Also add some useful compiler flag
	set(CMAKE_CXX_FLAGS
	   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x"
	)
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
	message(STATUS "Architecture is arm, compiling with NEON")
	# NEON flags
	add_definitions("-DENABLE_NEON")

	# Also add some useful compiler flag
	set(CMAKE_CXX_FLAGS
	   "${CMAKE_CXX_FLAGS} -O2 -march=native -mfpu=neon -std=c++0x"
	)  
endif()

# Set source files
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  #${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSImageStreamThread.cpp
  #${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/OpenCVImageStreamThread.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/OpenCVOutput3DWrapper.cpp
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)


# build shared library.
add_library(lsdslam SHARED ${SOURCE_FILES})
# target_link_libraries(lsdslam ${FABMAP_LIB} g2o_core g2o_stuff csparse cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba X11)

# target_link_libraries(lsdslam ${FABMAP_LIB} g2o_core g2o_stuff cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba ${Boost_LIBRARIES} opencv_core opencv_highgui opencv_imgproc opencv_calib3d)
target_link_libraries(lsdslam ${FABMAP_LIB} g2o_core g2o_stuff cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})


# build live ros node
# add_executable(live_slam src/main_live_odometry.cpp)
# target_link_libraries(live_slam lsdslam)


# build image node
# add_executable(dataset_slam src/main_on_images.cpp)
# target_link_libraries(dataset_slam lsdslam)

# build image node
add_executable(opencv_slam src/main_opencv.cpp)
target_link_libraries(opencv_slam lsdslam X11)
