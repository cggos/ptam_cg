cmake_minimum_required( VERSION 3.2 )
project ( PTAM4AR )

set( CMAKE_BUILD_TYPE "Release/Debug" )

set( CMAKE_CXX_COMPILER "g++" )

message(STATUS "CMAKE_CXX_COMPILER_VERSION: " ${CMAKE_CXX_COMPILER_VERSION})
if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.1)
    message(FATAL_ERROR "Too low CXX Version, it must equal or great than 5.1")
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
    message(FATAL_ERROR "CXX does not support -std=c++11")
endif()

message(STATUS "My C++ compiler supports these C++ features:")
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
    message(STATUS "  ${i}")
endforeach()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(MY_OVERALL_CXX_FLAGS    "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -Wall -lpthread -lm")
    set(CMAKE_CXX_FLAGS_RELEASE "${MY_OVERALL_CXX_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS_DEBUG   "${MY_OVERALL_CXX_FLAGS} -O0 -g -ggdb")# debug, no optimisation
    set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} --coverage -lgcov") # enabling coverage
endif()

# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin  )


############### set SRC_LIST_PTAM & SRC_LIST_CameraCalibrator ######################

set( VIDEOSOURCE
        ${PROJECT_SOURCE_DIR}/src/VideoSource_Linux_DataSet.cc
        ${PROJECT_SOURCE_DIR}/src/VideoSource_Linux_V4L.cc
        ${PROJECT_SOURCE_DIR}/src/VideoSource_Linux_DV.cc )

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


############### Static Analysis of C/C++ using cppcheck ######################

# generating compile_commands.json(JSON compilation database)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
# execute `make cppcheck` after `cmake`
include(cmake_modules/cppcheck.cmake)


############### coveralls-cmake: test code coverage ######################

# Add coveralls-cmake project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/coveralls-cmake/cmake)

# We probably don't want this to run on every build.
option(COVERALLS "Generate coveralls data" OFF)
if (COVERALLS)
    include(Coveralls)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -g -O0 -fprofile-arcs -ftest-coverage")
    # Create the coveralls target.
    coveralls_setup( "${SRC_LIST_PTAM}" ON )
endif()


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

# TooN
# Require linking the lapack library

# libCVD
link_libraries( cvd )

# GVars3
link_libraries( GVars3 )


############### project: PTAM & CameraCalibrator ######################

include_directories( "${PROJECT_SOURCE_DIR}/include" )

### PTAM
add_executable( PTAM ${SRC_LIST_PTAM} )

### CameraCalibrator
add_executable( CameraCalibrator ${SRC_LIST_CameraCalibrator} )


############### output some messages ######################

message("CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_CXX_FLAGS_RELEASE})
message("CMAKE_CXX_FLAGS_DEBUG  : " ${CMAKE_CXX_FLAGS_DEBUG})

message(${CMAKE_CURRENT_BINARY_DIR})
message(${CMAKE_BINARY_DIR})
message(${PROJECT_BINARY_DIR})
