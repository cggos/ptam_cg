// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "VideoSource.h"
#include "GLWindow2.h"

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;

class System
{
public:
    System();
    ~System();
    void Run();
    void UpdateFrame(CVD::Image<CVD::byte> imBW, CVD::Image<CVD::Rgb<CVD::byte> > imRGB);

private:
    VideoSource *mpVideoSource;
    GLWindow2 mGLWindow;

    Map *mpMap;
    MapMaker *mpMapMaker;
    Tracker *mpTracker;
    ATANCamera *mpCamera;
    ARDriver *mpARDriver;
    MapViewer *mpMapViewer;

    bool mbDone;

    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};



#endif
