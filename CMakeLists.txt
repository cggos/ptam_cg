cmake_minimum_required( VERSION 3.3 )
project ( PTAM_linux )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_COMPILER "g++" )
if(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG   "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -O0 -Wall -lpthread -lm -g -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -O3 -Wall -lpthread -lm")
endif()

message(${CMAKE_CXX_FLAGS_RELEASE})

message(${CMAKE_CURRENT_BINARY_DIR})
message(${CMAKE_BINARY_DIR})
message(${PROJECT_BINARY_DIR})

# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin  )

# add cmake modules
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

############### dependencies ######################

## Boost
#set(Boost_USE_STATIC_LIBS  OFF)
#set(Boost_USE_MULTITHREADED ON)
#find_package(Boost COMPONENTS system thread REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    link_libraries( ${OPENGL_LIBRARY} )
endif()

## GLUT
#find_package(GLUT REQUIRED)
#if(GLUT_FOUND)
#    link_libraries( ${GLUT_LIBRARY} )
#endif()

## GLEW
#find_package(GLEW REQUIRED)
#if (GLEW_FOUND)
#    include_directories(${GLEW_INCLUDE_DIRS})
#    link_libraries(${GLEW_LIBRARIES})
#endif()

# Eigen
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
    include_directories( ${EIGEN3_INCLUDE_DIR} )
endif()

## lapack
link_libraries( lapack )
#??????????????????????????????????????????????
#find_package(LAPACK REQUIRED)
#if(LAPACK_FOUND)
#    link_libraries( ${LAPACK_LIBRARIES} )
#    message(STATUS "    LAPACK Libraries: ")
#    foreach (lib ${LAPACK_LIBRARIES})
#        message(STATUS "        " ${lib})
#    endforeach()
#endif()

## Sophus
#find_package( Sophus REQUIRED )
#if(Sophus_FOUND)
#    include_directories( ${Sophus_INCLUDE_DIRS} )
#    link_libraries( ${Sophus_LIBRARIES} )
#endif()

# TooN
# Require linking the lapack library

# libCVD
link_libraries( cvd )

# GVars3
link_libraries( GVars3 )

## Ceres
#find_package( Ceres REQUIRED )
#if(Ceres_FOUND)
#    include_directories( ${CERES_INCLUDE_DIRS} )
#    link_libraries( ${CERES_LIBRARIES} )
#endif()

## G2O
#find_package( G2O REQUIRED )
#if(G2O_FOUND)
#    include_directories( ${G2O_INCLUDE_DIRS} )
#    link_libraries( g2o_core g2o_stuff g2o_types_sba )
#endif()

## Pangolin
#find_package( Pangolin )
#if(Pangolin_FOUND)
#    include_directories( ${Pangolin_INCLUDE_DIRS} )
#    link_directories( ${Pangolin_LIBRARIES} )
#endif()

## OpenCV
#find_package( OpenCV 3.1 REQUIRED )
#if(OpenCV_FOUND)
#    include_directories( ${OpenCV_INCLUDE_DIRS} )
#    link_libraries( ${OpenCV_LIBS} )
#endif()

## pcl
#set( PCL_DIR "/usr/local/share/pcl-1.7/" )
#find_package( PCL REQUIRED COMPONENTS common io )
#if(PCL_FOUND)
#    include_directories( ${PCL_INCLUDE_DIRS} )
#    add_definitions( ${PCL_DEFINITIONS} )
#    link_libraries( ${PCL_LIBRARIES} )
#endif()


############### Generating API documentation with Doxygen ######################

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
endif()


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
