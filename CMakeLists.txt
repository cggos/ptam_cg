cmake_minimum_required( VERSION 3.3 )
project ( PTAM_linux )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-D_LINUX -D_REENTRANT -Wall -std=c++11 -march=nocona -O3 -msse3" )# -march=native


# add cmake modules
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
#set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################

# Boost
set(Boost_USE_STATIC_LIBS  OFF)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost COMPONENTS system thread REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
link_libraries( ${OPENGL_LIBRARY} GLU glut )

# Eigen
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

# lapack
find_package(LAPACK REQUIRED)

## Sophus
#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )
#link_libraries( ${Sophus_LIBRARIES} )

# TooN
link_libraries( lapack )

# libCVD
link_libraries( cvd )

# GVars3
link_libraries( GVars3 )

## Ceres
#find_package( Ceres REQUIRED )
#include_directories( ${CERES_INCLUDE_DIRS} )
#link_libraries( ${CERES_LIBRARIES} )

## G2O
#find_package( G2O REQUIRED )
#include_directories( ${G2O_INCLUDE_DIRS} )
#link_libraries( g2o_core g2o_stuff g2o_types_sba )

## Pangolin
#find_package( Pangolin )
#include_directories( ${Pangolin_INCLUDE_DIRS} )
#link_directories( "/usr/local/lib/" )

## OpenCV
#find_package( OpenCV 3.1 REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#link_libraries( ${OpenCV_LIBS} )

## pcl
#set( PCL_DIR "/usr/local/share/pcl-1.7/" )
#find_package( PCL REQUIRED COMPONENTS common io )
#include_directories( ${PCL_INCLUDE_DIRS} )
#add_definitions( ${PCL_DEFINITIONS} )
#link_libraries( ${PCL_LIBRARIES} )


############### project: PTAM & CameraCalibrator ######################

include_directories( "${PROJECT_SOURCE_DIR}/include" )

# file( GLOB SRC_CC_LIST "${PROJECT_SOURCE_DIR}/src/*.cc")
# aux_source_directory( ${PROJECT_SOURCE_DIR}/src SRC_LIST)

# VideoSource_Linux_DV.cc      --> VideoSource.h
# VideoSource_Linux_V4L.cc     --> VideoSource.h
# VideoSource_Linux_DataSet.cc --> VideoSource_Linux_DataSet.h
set( VIDEOSOURCE ${PROJECT_SOURCE_DIR}/src/VideoSource_Linux_DataSet.cc )

### PTAM
add_executable( PTAM
        ${PROJECT_SOURCE_DIR}/src/main.cc
        ${PROJECT_SOURCE_DIR}/src/GLWindow2.cc
        ${PROJECT_SOURCE_DIR}/src/GLWindowMenu.cc
        ${PROJECT_SOURCE_DIR}/src/System.cc
        ${PROJECT_SOURCE_DIR}/src/ATANCamera.cc
        ${PROJECT_SOURCE_DIR}/src/KeyFrame.cc
        ${PROJECT_SOURCE_DIR}/src/MapPoint.cc
        ${PROJECT_SOURCE_DIR}/src/Map.cc
        ${PROJECT_SOURCE_DIR}/src/SmallBlurryImage.cc
        ${PROJECT_SOURCE_DIR}/src/ShiTomasi.cc
        ${PROJECT_SOURCE_DIR}/src/HomographyInit.cc
        ${PROJECT_SOURCE_DIR}/src/MapMaker.cc
        ${PROJECT_SOURCE_DIR}/src/Bundle.cc
        ${PROJECT_SOURCE_DIR}/src/PatchFinder.cc
        ${PROJECT_SOURCE_DIR}/src/Relocaliser.cc
        ${PROJECT_SOURCE_DIR}/src/MiniPatch.cc
        ${PROJECT_SOURCE_DIR}/src/MapViewer.cc
        ${PROJECT_SOURCE_DIR}/src/ARDriver.cc
        ${PROJECT_SOURCE_DIR}/src/EyeGame.cc
        ${PROJECT_SOURCE_DIR}/src/Tracker.cc
        ${VIDEOSOURCE} )
# target_link_libraries( PTAM ${THIRD_PARTY_LIBS} )

#### CameraCalibrator
#add_executable( CameraCalibrator
#        ${PROJECT_SOURCE_DIR}/src/GLWindow2.cc
#        ${PROJECT_SOURCE_DIR}/src/GLWindowMenu.cc
#        ${PROJECT_SOURCE_DIR}/src/CalibImage.cc
#        ${PROJECT_SOURCE_DIR}/src/CalibCornerPatch.cc
#        ${PROJECT_SOURCE_DIR}/src/ATANCamera.cc
#        ${PROJECT_SOURCE_DIR}/src/CameraCalibrator.cc
#        ${VIDEOSOURCE} )
## target_link_libraries( CameraCalibrator ${THIRD_PARTY_LIBS} )
