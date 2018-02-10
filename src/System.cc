// Copyright 2008 Isis Innovation Limited

#include "System.h"

#include "OpenGL.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

System::System()
    : mpVideoSource(new VideoSourceV4L())
    , mGLWindow(mpVideoSource->Size(), "PTAM")
{
    GUI.RegisterCommand("exit", GUICommandCallBack, this);
    GUI.RegisterCommand("quit", GUICommandCallBack, this);

    // First, check if the camera is calibrated.
    // If not, we need to run the calibration widget.
    Vector<NUMTRACKERCAMPARAMETERS> vTest;
    vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
    if(vTest == ATANCamera::mvDefaultParams)
    {
        cout << endl;
        cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
        cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
        exit(1);
    }

    mpCamera = new ATANCamera("Camera");
    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera);
    mpTracker = new Tracker(mpVideoSource->Size(), *mpCamera, *mpMap, *mpMapMaker);
    mpARDriver = new ARDriver(*mpCamera, mpVideoSource->Size(), mGLWindow);
    mpMapViewer = new MapViewer(*mpMap, mGLWindow);

    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
    GUI.ParseLine("DrawAR=0");
    GUI.ParseLine("DrawMap=0");
    GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
    GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

    mbDone = false;
}

void System::Run()
{
    while(!mbDone)
    {
        CVD::Image<CVD::Rgb<CVD::byte> > imFrameRGB(mpVideoSource->Size());
        CVD::Image<CVD::byte> imFrameBW(mpVideoSource->Size());

        mpVideoSource->GetAndFillFrameBWandRGB(imFrameBW, imFrameRGB);

        UpdateFrame(imFrameBW, imFrameRGB);
    }
}

/**
 * @brief update System with every frame or image
 * @param imBW   gray image
 * @param imRGB  color image
 */
void System::UpdateFrame(Image<byte> imBW, Image<Rgb<byte> > imRGB)
{
    static bool bFirstFrame = true;
    if(bFirstFrame)
    {
        mpARDriver->Init();
        bFirstFrame = false;
    }

    mGLWindow.SetupViewport();
    mGLWindow.SetupVideoOrtho();
    mGLWindow.SetupVideoRasterPosAndZoom();

    if(!mpMap->IsGood())
        mpARDriver->Reset();

    static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
    static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);

    bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
    bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;

    mpTracker->TrackFrame(imBW, !bDrawAR && !bDrawMap);

    if(bDrawMap)
        mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
    else if(bDrawAR)
        mpARDriver->Render(imRGB, mpTracker->GetCurrentPose());

    string sCaption;
    if(bDrawMap)
        sCaption = mpMapViewer->GetMessageForUser();
    else
        sCaption = mpTracker->GetMessageForUser();
    mGLWindow.DrawCaption(sCaption);
    mGLWindow.DrawMenus();
    mGLWindow.swap_buffers();
    mGLWindow.HandlePendingEvents();
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
    if(sCommand=="quit" || sCommand == "exit")
        static_cast<System*>(ptr)->mbDone = true;
}

System::~System()
{
    if(mpVideoSource!=NULL)
    {
        delete mpVideoSource;
        mpVideoSource = NULL;
    }
}
