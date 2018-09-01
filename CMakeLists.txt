cmake_minimum_required( VERSION 3.2 )
project ( ptam_cg )

set(VERSION_MAJOR 1)
set(VERSION_MINOR 4)
set(VERSION_PATCH 0)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()


############### set SRC_LIST_PTAM & SRC_LIST_CameraCalibrator ######################

file(GLOB_RECURSE ALL_SOURCE_FILES ${PROJECT_SOURCE_DIR} *.cc *.h)

include_directories( "${PROJECT_SOURCE_DIR}/include" )

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

message(STATUS "My C++ compiler supports these C++ features:")
foreach(feature ${CMAKE_CXX_COMPILE_FEATURES})
    message(STATUS "  ${feature}")
endforeach()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")

    set(MY_OVERALL_CXX_FLAGS "$ENV{CXXFLAGS} -D_LINUX -D_REENTRANT -std=c++11 -march=nocona -msse3 -Wno-deprecated-declarations -lpthread -lm")

    if(CMAKE_BUILD_TYPE MATCHES "Release")

        set(CMAKE_CXX_FLAGS_RELEASE "${MY_OVERALL_CXX_FLAGS} -O3")

        set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})

    elseif(CMAKE_BUILD_TYPE MATCHES "Debug")

        add_definitions(-D__USER_DEBUG__)

        set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE OFF)

        set(CMAKE_CXX_FLAGS_DEBUG   "${MY_OVERALL_CXX_FLAGS} -g -O0 -Wall")

        if(NOT WIN32 AND NOT APPLE)

            message(STATUS "*****")
            message(STATUS "***** CXX_FLAGS enabled for gprof,")
            message(STATUS "***** generate a gmon.out file after executing project program successfully")
            message(STATUS "***** run `gprof <program_name> gmon.out`")
            set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -pg")

            message(STATUS "*****")
            message(STATUS "***** CXX_FLAGS enabled for gcov(debug, Linux builds only)")
            message(STATUS "***** generate .gcno files after `make`")
            message(STATUS "***** generate .gcda files after executing <project_program> successfully")
            set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG}  -fprofile-arcs -ftest-coverage")
            set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage -lgcov")

            message(STATUS "*****")
            message(STATUS "***** Create the gcov target for program ${CMAKE_PROJECT_NAME}")
            message(STATUS "***** run `make gcov` after executing ${CMAKE_PROJECT_NAME}")
            add_custom_target(gcov
                COMMAND mkdir -p gcov_coverage_test
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            )
            add_custom_command(TARGET gcov
                COMMAND echo "=================== GCOV ===================="
                COMMAND gcov -b ${CMAKE_BINARY_DIR}/CMakeFiles/${CMAKE_PROJECT_NAME}.dir/src/*.gcno
                COMMAND echo "-- Coverage files have been output to ${CMAKE_BINARY_DIR}/gcov_coverage_test"
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/gcov_coverage_test
            )
            add_dependencies(gcov ${CMAKE_PROJECT_NAME})
            # Make sure to clean up the coverage folder
            set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES gcov_coverage_test)

        endif()

        message(STATUS "*****")
        message(STATUS "***** Static Analysis of C/C++ using cppcheck")
        find_program(CPPCHECK NAMES cppcheck)
        if(CPPCHECK)
            message(STATUS "***** Found cppcheck: ${CPPCHECK}")

            message(STATUS "*****")
            message(STATUS "***** generate compile_commands.json(JSON compilation database)")
            message(STATUS "***** run `cppcheck --project=compile_commands.json` ")
            set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

            message(STATUS "*****")
            message(STATUS "***** run `make cppcheck` after `cmake`")
            add_custom_target(
                    cppcheck
                    COMMAND /usr/bin/cppcheck
                    --enable=warning,performance,portability,information,missingInclude
                    --std=c++11
                    --template="[{severity}][{id}] {message} {callstack} \(On {file}:{line}\)"
                    --verbose
                    --quiet
                    ${ALL_SOURCE_FILES}
            )
        else()
            message(WARNING "***** NOT Found cppcheck !!!")
        endif()

        set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

        message(STATUS "*****")

    endif(CMAKE_BUILD_TYPE MATCHES "Release")

endif()


############### Generating API documentation with Doxygen ######################
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" OFF)

if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "NOT Found doxygen which is needed to build the documentation.")
    else()
        message(STATUS "***** Found doxygen, run `make doc` to build documentation")
        set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/config/Doxyfile.config)
        set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        configure_file(${doxyfile_in} ${doxyfile} @ONLY)
        add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
    endif()
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

add_executable( ${CMAKE_PROJECT_NAME} ${SRC_LIST_PTAM} )

add_executable( ${CMAKE_PROJECT_NAME}_cam_calib ${SRC_LIST_CameraCalibrator} )


############### output some messages ######################
message(STATUS "")

message(STATUS "CMake Build Type: " ${CMAKE_BUILD_TYPE})

message(STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})

message(STATUS "CMAKE_CURRENT_BINARY_DIR: " ${CMAKE_CURRENT_BINARY_DIR})
message(STATUS "CMAKE_BINARY_DIR:         " ${CMAKE_BINARY_DIR})
message(STATUS "PROJECT_BINARY_DIR:       " ${PROJECT_BINARY_DIR})

message(STATUS "")
