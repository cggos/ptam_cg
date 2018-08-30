cmake_minimum_required( VERSION 3.2 )
project ( ptam_cg )

set(VERSION_MAJOR 1)
set(VERSION_MINOR 3)
set(VERSION_PATCH 0)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

message("CMake Build Type: " ${CMAKE_BUILD_TYPE})

set(USER_USE_GCOV     TRUE) # gcov
set(USER_USE_GPROF    TRUE) # gprof
set(USER_USE_CPPCHECK TRUE) # cppcheck
set(USER_USE_DOXYGEN  TRUE) # Doxygen


############### set SRC_LIST_PTAM & SRC_LIST_CameraCalibrator ######################

set( SRC_LIST_PTAM
        ${PROJECT_SOURCE_DIR}/src/main.cc
        ${PROJECT_SOURCE_DIR}/src/GLWindow2.cc
        ${PROJECT_SOURCE_DIR}/src/System.cc
        ${PROJECT_SOURCE_DIR}/src/ATANCamera.cc
        ${PROJECT_SOURCE_DIR}/src/KeyFrame.cc
        ${PROJECT_SOURCE_DIR}/src/Map.cc
        ${PROJECT_SOURCE_DIR}/src/HomographyInit.cc
        ${PROJECT_SOURCE_DIR}/src/MapMaker.cc
        ${PROJECT_SOURCE_DIR}/src/Bundle.cc
        ${PROJECT_SOURCE_DIR}/src/PatchFinder.cc
        ${PROJECT_SOURCE_DIR}/src/Relocaliser.cc
        ${PROJECT_SOURCE_DIR}/src/MapViewer.cc
        ${PROJECT_SOURCE_DIR}/src/ARDriver.cc
        ${PROJECT_SOURCE_DIR}/src/EyeGame.cc
        ${PROJECT_SOURCE_DIR}/src/Tracker.cc
        ${PROJECT_SOURCE_DIR}/src/Tools.cc
        ${PROJECT_SOURCE_DIR}/src/VideoSource.cc
        ${PROJECT_SOURCE_DIR}/src/ImageProcess.cc )

set( SRC_LIST_CameraCalibrator
        ${PROJECT_SOURCE_DIR}/src/GLWindow2.cc
        ${PROJECT_SOURCE_DIR}/src/CalibImage.cc
        ${PROJECT_SOURCE_DIR}/src/CalibCornerPatch.cc
        ${PROJECT_SOURCE_DIR}/src/ATANCamera.cc
        ${PROJECT_SOURCE_DIR}/src/CameraCalibrator.cc
        ${PROJECT_SOURCE_DIR}/src/VideoSource.cc )


############### set CMake Compiler ######################

set( CMAKE_CXX_COMPILER "g++" )

# check compiler version > 5.1
message(STATUS "CMAKE_CXX_COMPILER_VERSION: " ${CMAKE_CXX_COMPILER_VERSION})
if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.1)
    message(FATAL_ERROR "Too low CXX Version, it must equal or great than 5.1")
endif()

# check and set C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
    message(FATAL_ERROR "CXX does not support -std=c++11")
endif()

# output C++ features
message(STATUS "My C++ compiler supports these C++ features:")
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
    message(STATUS "  ${i}")
endforeach()

# set CMAKE_CXX_FLAGS
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")

    set(MY_OVERALL_CXX_FLAGS    "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -Wall -Wno-deprecated-declarations -lpthread -lm")

    if(CMAKE_BUILD_TYPE MATCHES "Release")

        set(CMAKE_CXX_FLAGS_RELEASE "${MY_OVERALL_CXX_FLAGS} -O3")
        set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})

    else(CMAKE_BUILD_TYPE MATCHES "Debug")
        set(CMAKE_CXX_FLAGS_DEBUG   "${MY_OVERALL_CXX_FLAGS} -g")

        if(USER_USE_GPROF)
            set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -pg") # enabling gprof
        endif()

        if(USER_USE_GCOV)
            ############### coveralls-cmake: test code coverage ######################
            # Add coveralls-cmake project cmake modules to path.
            set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/coveralls-cmake/cmake)
            # We probably don't want this to run on every build.
            option(COVERALLS "Generate coveralls data" OFF)
            if (COVERALLS)
                include(Coveralls)
                #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} --coverage -lgcov") # enabling coverage gcov
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage -lgcov")
                # Create the coveralls target.
                coveralls_setup( "${SRC_LIST_PTAM}" ON )
            endif()
        endif()

        set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

        add_definitions(-D__USER_DEBUG__)

    endif(CMAKE_BUILD_TYPE MATCHES "Release")

endif()


# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin  )


############### Static Analysis of C/C++ using cppcheck ######################
if (USER_USE_CPPCHECK)

    # generating compile_commands.json(JSON compilation database)
    set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

    find_program(CPPCHECK NAMES cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck: ${CPPCHECK}")
    else()
        message(WARNING "NOT Found cppcheck")
    endif()

    # execute `make cppcheck` after `cmake`
    include(cmake_modules/cppcheck.cmake)

endif ()


############### Generating API documentation with Doxygen ######################
if (USER_USE_DOXYGEN)
    # add a target to generate API documentation with Doxygen
    find_package(Doxygen)
    option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

    if(BUILD_DOCUMENTATION)
        if(NOT DOXYGEN_FOUND)
            message(FATAL_ERROR "Doxygen is needed to build the documentation.")
        endif()

        set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/Doxyfile.config)
        set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        configure_file(${doxyfile_in} ${doxyfile} @ONLY)

        add_custom_target(doc
                COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM)

        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
    endif()
endif ()


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

message("CMAKE_CXX_FLAGS  : " ${CMAKE_CXX_FLAGS})

message("CMAKE_CURRENT_BINARY_DIR: " ${CMAKE_CURRENT_BINARY_DIR})
message("CMAKE_BINARY_DIR:         " ${CMAKE_BINARY_DIR})
message("PROJECT_BINARY_DIR:       " ${PROJECT_BINARY_DIR})
