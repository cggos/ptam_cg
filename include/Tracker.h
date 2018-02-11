//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include <sstream>
#include <vector>
#include <list>

#include "MapMaker.h"
#include "ATANCamera.h"
#include "ImageProcess.h"
#include "Relocaliser.h"
#include "PatchFinder.h"

// This class contains all the intermediate results associated with
// a map-point that the tracker keeps up-to-date. TrackerData
// basically handles all the tracker's point-projection jobs,
// and also contains the PatchFinder which does the image search.
// It's very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

struct TrackerData
{
    TrackerData(MapPoint *pMapPoint):Point(*pMapPoint){}

    MapPoint &Point;
    PatchFinder Finder;

    // Projection itermediates:
    Vector<3> v3Cam;        // Coords in current cam frame
    Vector<2> v2ImPlane;    // Coords in current cam z=1 plane
    Vector<2> v2Image;      // Pixel coords in LEVEL0
    Matrix<2> m2CamDerivs;  // Camera projection derivs
    bool bInImage;
    bool bPotentiallyVisible;

    int nSearchLevel;
    bool bSearched;
    bool bFound;
    bool bDidSubPix;
    Vector<2> v2Found;      // Pixel coords of found patch (L0)
    double dSqrtInvNoise;   // Only depends on search level..

    // Stuff for pose update:
    Vector<2> v2Error_CovScaled;
    Matrix<2,6> m26Jacobian;   // Jacobian wrt camera position

    // Project point into image given certain pose and camera.
    // This can bail out at several stages if the point
    // will not be properly in the image.
    inline void Project(const SE3<> &se3CFromW, ATANCamera &Cam)
    {
        bInImage = bPotentiallyVisible = false;
        v3Cam = se3CFromW * Point.v3WorldPos;
        if(v3Cam[2] < 0.001)
            return;
        v2ImPlane = project(v3Cam);
        if(v2ImPlane*v2ImPlane > Cam.LargestRadiusInImage() * Cam.LargestRadiusInImage())
            return;
        v2Image = Cam.Project(v2ImPlane);
        if(Cam.Invalid())
            return;

        if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
            return;
        bInImage = true;
    }

    // Does projection and gets camera derivs all in one.
    inline void ProjectAndDerivs(SE3<> &se3, ATANCamera &Cam)
    {
        Project(se3, Cam);
        if(bFound)
            m2CamDerivs = Cam.GetProjectionDerivs();
    }

    /**
     * @brief Jacobian of projection W.R.T. the camera position
     *        I.e. if  p_cam = SE3Old * p_world,
     *                 SE3New = SE3Motion * SE3Old
     * @details
     *         \f[
     *
     *           J_2 = \frac{\partial{(x_c,y_c)}}{\partial{(X_c,Y_c,Z_c)}}
     *               = \begin{bmatrix}
     *                       \frac{1}{Z_c} & 0 & -\frac{X_c}{{Z_c}^2} \\
     *                       0 & \frac{1}{Z_c} & -\frac{Y_c}{{Z_c}^2}
     *                 \end{bmatrix}
     *               = \begin{bmatrix}
     *                       1 & 0 & -\frac{X_c}{Z_c} \\
     *                       0 & 1 & -\frac{Y_c}{Z_c}
     *                 \end{bmatrix}
     *                 * \frac{1}{Z_c} \\
     *
     *           J_3 = \frac{\partial{(X_c,Y_c,Z_c)}}{\partial{\xi}}
     *               = \begin{bmatrix}
     *                       1 & 0 & 0 &   0  &  Z_c &  -Y_c \\
     *                       0 & 1 & 0 & -Z_c &   0  &   X_c \\
     *                       0 & 0 & 1 &  Y_c & -X_c &    0
     *                 \end{bmatrix} \\
     *
     *             J = J_1 \cdot J_2 \cdot J_3
     *
     *         \f]
     */
    inline void CalcJacobian()
    {
        double dOneOverCameraZ = 1.0 / v3Cam[2];
        for(int m=0; m<6; m++)
        {
            const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
            Vector<2> v2CamFrameMotion; //J_2 * J_3
            v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
            v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
            m26Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion; // J = J_1 * J_2 * J_3
        }
    }

    // Sometimes in tracker instead of reprojecting, just update the error linearly!
    inline void LinearUpdate(const Vector<6> &v6)
    {
        v2Image += m26Jacobian * v6;
    }

    // This static member is filled in by the tracker and allows in-image checks in this class above.
    static CVD::ImageRef irImageSize;
};

struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
    MiniPatch mPatch;
    CVD::ImageRef irCurrentPos;
    CVD::ImageRef irInitialPos;
};

class Tracker
{
public:
    Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);

    // TrackFrame is the main working part of the tracker: call this every frame.
    void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw);

    inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}

    // Gets messages to be printed on-screen for the user.
    std::string GetMessageForUser();

protected:
    KeyFrame mCurrentKF;            // The current working frame as a keyframe struct

    // The major components to which the tracker needs access:
    Map &mMap;                      // The map, consisting of points and keyframes
    MapMaker &mMapMaker;            // The class which maintains the map
    ATANCamera mCamera;             // Projection model
    Relocaliser mRelocaliser;       // Relocalisation module

    CVD::ImageRef mirSize;          // Image size of whole image

    void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
    void RenderGrid();              // Draws the reference grid

    // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
    void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
    enum {TRAIL_TRACKING_NOT_STARTED,
          TRAIL_TRACKING_STARTED,
          TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
    void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
    int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
    std::list<Trail> mlTrails;      // Used by trail tracking
    KeyFrame mFirstKF;              // First of the stereo pair
    KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches

    // Methods for tracking the map once it has been made:
    void TrackMap();                // Called by TrackFrame if there is a map.
    void TrackMapLocal();
    void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
    void PredictPoseWithMotionModel();        // Decaying velocity motion model applied prior to TrackMap
    void UpdateMotionModel();       // Motion model is updated after TrackMap
    unsigned int SearchForPoints(std::vector<TrackerData*> &vTD, unsigned int nRange, int nFineIts);  // Finds points in the image
    Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD,
                             double dOverrideSigma = 0.0,
                             bool bMarkOutliers = false); // Updates pose from found points.
    SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
    SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
    Vector<6> mv6CameraVelocity;    // Motion model
    double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
    bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

    bool mbDraw;                    // Should the tracker draw anything to OpenGL?

    // Interface with map maker:
    int mnFrame;                    // Frames processed since last reset
    int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.

    // Tracking quality control:
    int manMeasAttempted[LEVELS];
    int manMeasFound[LEVELS];
    enum {BAD, GOOD} mTrackingQuality;
    int mnLostFrames;

    // Relocalisation functions:
    bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
    bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

    // Frame-to-frame motion init:
    SmallBlurryImage *mpSBILastFrame;
    SmallBlurryImage *mpSBIThisFrame;
    bool mbUseSBIInit;

    // User interaction for initial tracking:
    bool mbUserPressedSpacebar;
    std::ostringstream mMessageForUser;

    // GUI interface:
    void GUICommandHandler(std::string sCommand, std::string sParams);
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
    struct Command {std::string sCommand; std::string sParams; };
    std::vector<Command> mvQueuedCommands;
};

#endif
