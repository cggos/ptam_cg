cmake_minimum_required( VERSION 3.2 )
project ( PTAM_linux )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_COMPILER "g++-5" )
if(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG   "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -O0 -Wall -lpthread -lm -g -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -O3 -Wall -lpthread -lm")
endif()

message(${CMAKE_CXX_FLAGS_RELEASE})

message(${CMAKE_CURRENT_BINARY_DIR})
message(${CMAKE_BINARY_DIR})
message(${PROJECT_BINARY_DIR})

# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin  )

############### Dependencies or AddOns ######################

# add cmake modules
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    link_libraries( ${OPENGL_LIBRARY} )
endif()

# lapack
link_libraries( lapack )
#find_package(LAPACK REQUIRED)
#if(LAPACK_FOUND)
#    message(STATUS "LAPACK Libraries: ")
#    foreach (lib ${LAPACK_LIBRARIES})
#        message(STATUS "  " ${lib})
#    endforeach()
#    link_libraries( ${LAPACK_LIBRARIES} )
#endif()

# TooN
# Require linking the lapack library

# libCVD
link_libraries( cvd )

# GVars3
link_libraries( GVars3 )


############### Static Analysis of C/C++ using cppcheck ######################
# generating compile_commands.json(JSON compilation database)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
# execute `make cppcheck` after `cmake`
include(cmake_modules/cppcheck.cmake)

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

set( SRC_LIST_PTAM
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

set( SRC_LIST_CameraCalibrator
        ${PROJECT_SOURCE_DIR}/src/GLWindow2.cc
        ${PROJECT_SOURCE_DIR}/src/GLWindowMenu.cc
        ${PROJECT_SOURCE_DIR}/src/CalibImage.cc
        ${PROJECT_SOURCE_DIR}/src/CalibCornerPatch.cc
        ${PROJECT_SOURCE_DIR}/src/ATANCamera.cc
        ${PROJECT_SOURCE_DIR}/src/CameraCalibrator.cc
        ${VIDEOSOURCE} )

### PTAM
add_executable( PTAM ${SRC_LIST_PTAM} )

### CameraCalibrator
#add_executable( CameraCalibrator ${SRC_LIST_CameraCalibrator} )
