// Copyright 2008 Isis Innovation Limited

#include "Tracker.h"

#include <fstream>
#include <fcntl.h>
#include <unistd.h>

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include "OpenGL.h"
#include "Tools.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm) : 
    mMap(m),
    mMapMaker(mm),
    mCamera(c),
    mRelocaliser(mMap, mCamera),
    mirSize(irVideoSize)
{
    mCurrentKF.bFixed = false;
    GUI.RegisterCommand("Reset", GUICommandCallBack, this);
    GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
    GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
    TrackerData::irImageSize = mirSize;

    mpSBILastFrame = nullptr;
    mpSBIThisFrame = nullptr;

    // Most of the initialisation is done in Reset()
    Reset();
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{
    mbDidCoarse = false;
    mbUserPressedSpacebar = false;
    mTrackingQuality = GOOD;
    mnLostFrames = 0;
    mdMSDScaledVelocityMagnitude = 0;
    mCurrentKF.dSceneDepthMean = 1.0;
    mCurrentKF.dSceneDepthSigma = 1.0;
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
    mCamera.SetImageSize(mirSize);
    mCurrentKF.mMeasurements.clear();
    mnLastKeyFrameDropped = -20;
    mnFrame=0;
    mv6CameraVelocity = Zeros;
    mbJustRecoveredSoUseCoarse = false;

    // Tell the MapMaker to reset itself..
    // this may take some time, since the mapmaker thread may have to wait
    // for an abort-check during calculation, so sleep while waiting.
    // MapMaker will also clear the map.
    mMapMaker.RequestReset();
    while(!mMapMaker.ResetDone())
#ifndef WIN32
        usleep(10);
#else
        Sleep(1);
#endif
}

/**
 * @brief TrackFrame is called by System.cc with each incoming video frame.
 *        It figures out what state the tracker is in, and calls appropriate internal tracking functions.
 * @param imFrame gray image or frame
 * @param bDraw bDraw tells the tracker wether it should output any GL graphics or not
 */
void Tracker::TrackFrame(Image<byte> &imFrame, bool bDraw)
{
    mbDraw = bDraw;
    mMessageForUser.str("");

    mCurrentKF.mMeasurements.clear();
    mCurrentKF.MakeKeyFrame_Lite(imFrame);// This does things like generate the image pyramid and find FAST corners

    // Update the small images for the rotation estimator
    static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
    static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
    mbUseSBIInit = (bool)*gvnUseSBI;
    if(!mpSBIThisFrame)
    {
        mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
        mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
    else
    {
        delete  mpSBILastFrame;
        mpSBILastFrame = mpSBIThisFrame;
        mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }

    // From now on we only use the keyframe struct!
    mnFrame++;

    if(mbDraw)
    {
        glDrawPixels(mCurrentKF.aLevels[0].im);
        if(GV2.GetInt("Tracker.DrawFASTCorners",0, SILENT))
        {
            glColor3f(1,0,1);
            glPointSize(1);
            glBegin(GL_POINTS);
            for (const auto &vCorner : mCurrentKF.aLevels[0].vCorners)
                glVertex(vCorner);
            glEnd();
        }
    }

    if(mMap.IsGood())
    {
        if(mnLostFrames < 3)
        {
            PredictPoseWithMotionModel();
            TrackMap();
            UpdateMotionModel();

            AssessTrackingQuality();
            {
                mMessageForUser << "Tracking Map, quality ";
                if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
                if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
                mMessageForUser << " Found:";
                for(int i=0; i<LEVELS; i++)
                    mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
                mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
            }

            bool isTrackGood = mTrackingQuality == GOOD;
            bool isNeedFrame = mMapMaker.IsNeedNewKeyFrame(mCurrentKF);
            bool isFarEnough = mnFrame - mnLastKeyFrameDropped > 20;
            bool isQueueNeed = mMapMaker.QueueSize() < 3;

            if(isTrackGood)
                mMessageForUser << " 1";
            if(isNeedFrame)
                mMessageForUser << " 2";
            if(isFarEnough)
                mMessageForUser << " 3";
            if(isQueueNeed)
                mMessageForUser << " 4";

            // Heuristics to check if a key-frame should be added to the map:
            if( isTrackGood &&  isFarEnough && isQueueNeed )//isNeedFrame
            {
                mMessageForUser << " Adding key-frame.";
                mMapMaker.AddKeyFrame(mCurrentKF);
                mnLastKeyFrameDropped = mnFrame;
            }
        }
        else  // what if there is a map, but tracking has been lost?
        {
            mMessageForUser << "** Attempting recovery **.";
            if(AttemptRecovery())
            {
                TrackMap();
                AssessTrackingQuality();
            }
        }
        if(mbDraw)
            RenderGrid();
    }
    else
        TrackForInitialMap();

    while(!mvQueuedCommands.empty())
    {
        GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
        mvQueuedCommands.erase(mvQueuedCommands.begin());
    }
}

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
    bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
    if(!bRelocGood)
        return false;

    SE3<> se3Best = mRelocaliser.BestPose();
    mse3CamFromWorld = mse3StartPos = se3Best;
    mv6CameraVelocity = Zeros;
    mbJustRecoveredSoUseCoarse = true;
    return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void Tracker::RenderGrid()
{
    // The colour of the ref grid shows if the coarse stage of tracking was used
    // (it's turned off when the camera is sitting still to reduce jitter.)
    if(mbDidCoarse)
        glColor4f(.0, 0.5, .0, 0.6);
    else
        glColor4f(0,0,0,0.6);

    // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
    int nHalfCells = 8;
    int nTot = nHalfCells * 2 + 1;
    Image<Vector<2> >  imVertices(ImageRef(nTot,nTot));
    for(int i=0; i<nTot; i++)
        for(int j=0; j<nTot; j++)
        {
            Vector<3> v3;
            v3[0] = (i - nHalfCells) * 0.1;
            v3[1] = (j - nHalfCells) * 0.1;
            v3[2] = 0.0;
            Vector<3> v3Cam = mse3CamFromWorld * v3;
            if(v3Cam[2] < 0.001)
                v3Cam[2] = 0.001;
            imVertices[i][j] = mCamera.Project(project(v3Cam));
        }
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(2);
    for(int i=0; i<nTot; i++)
    {
        glBegin(GL_LINE_STRIP);
        for(int j=0; j<nTot; j++)
            glVertex(imVertices[i][j]);
        glEnd();

        glBegin(GL_LINE_STRIP);
        for(int j=0; j<nTot; j++)
            glVertex(imVertices[j][i]);
        glEnd();
    };

    glLineWidth(1);
    glColor3f(1,0,0);
}

// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
    Command c;
    c.sCommand = sCommand;
    c.sParams = sParams;
    ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
    if(sCommand=="Reset")
    {
        Reset();
        return;
    }

    // KeyPress commands are issued by GLWindow
    if(sCommand=="KeyPress")
    {
        if(sParams == "Space")
        {
            mbUserPressedSpacebar = true;
        }
        else if(sParams == "r")
        {
            Reset();
        }
        else if(sParams == "q" || sParams == "Escape")
        {
            GUI.ParseLine("quit");
        }
        return;
    }
    if((sCommand=="PokeTracker"))
    {
        mbUserPressedSpacebar = true;
        return;
    }
    

    cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
    exit(1);
}

/**
 * @brief Routine for establishing the initial map. This requires two spacebar presses from the user
 *        to define the first two key-frames. Salient points are tracked between the two keyframes
 *        using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
 *        break it.) The salient points are stored in a list of `Trail' data structures.
 *        What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
 */
void Tracker::TrackForInitialMap()
{
    if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
    {
        if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
        {
            mbUserPressedSpacebar = false;
            TrailTracking_Start();
            mnInitialStage = TRAIL_TRACKING_STARTED;
        }
        else
            mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map." << endl;
        return;
    };

    if(mnInitialStage == TRAIL_TRACKING_STARTED)
    {
        int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
        if(nGoodTrails < 10) {
            Reset();
            return;
        }

        // If the user pressed spacebar here, use trails to run stereo and make the intial map..
        if(mbUserPressedSpacebar)
        {
            mbUserPressedSpacebar = false;
            vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
            for (auto &mlTrail : mlTrails)
                vMatches.push_back(pair<ImageRef, ImageRef>(mlTrail.irInitialPos, mlTrail.irCurrentPos));
            mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
            mnInitialStage = TRAIL_TRACKING_COMPLETE;
        }
        else
            mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init." << endl;
    }
}

/**
 * @brief The current frame is to be the first keyframe!
 */
void Tracker::TrailTracking_Start()
{
    mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
    mFirstKF = mCurrentKF;
    vector<pair<double,ImageRef> > vCornersAndSTScores;
    for (auto &c : mCurrentKF.aLevels[0].vCandidates)
        vCornersAndSTScores.push_back(pair<double,ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
    sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
    int nToAdd = GV2.GetInt("Tracker.MaxInitialTrails", 1000, SILENT);
    for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++) {
        Trail t;
        t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
        t.irInitialPos = vCornersAndSTScores[i].second;
        t.irCurrentPos = t.irInitialPos;
        mlTrails.push_back(t);
        nToAdd--;
    }
    mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.
}

/**
 * @brief Steady-state trail tracking: Advance from the previous frame, remove duds.
 * @return
 */
int Tracker::TrailTracking_Advance()
{
    int nGoodTrails = 0;
    if(mbDraw) {
        glPointSize(5);
        glLineWidth(2);
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glBegin(GL_LINES);
    }

    MiniPatch BackwardsPatch;
    Level &lCurrentFrame = mCurrentKF.aLevels[0];
    Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

    for(auto i = mlTrails.begin(); i!=mlTrails.end();) {
        auto next = i;
        next++;

        Trail &trail = *i;
        ImageRef irStart = trail.irCurrentPos;
        ImageRef irEnd = irStart;
        bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
        if (bFound) {
            // Also find backwards in a married-matches check
            BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
            ImageRef irBackWardsFound = irEnd;
            bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
            if ((irBackWardsFound - irStart).mag_squared() > 2)
                bFound = false;

            trail.irCurrentPos = irEnd;
            nGoodTrails++;
        }
        if (mbDraw) {
            if (!bFound)
                glColor3f(0, 1, 1); // Failed trails flash purple before dying.
            else
                glColor3f(1, 1, 0);
            glVertex(trail.irInitialPos);
            if (bFound) glColor3f(1, 0, 0);
            glVertex(trail.irCurrentPos);
        }
        if (!bFound) // Erase from list of trails if not found this frame.
        {
            mlTrails.erase(i);
        }
        i = next;
    }
    if(mbDraw)
        glEnd();

    mPreviousFrameKF = mCurrentKF;
    return nGoodTrails;
}

/**
 * @details TrackMap is the main purpose of the Tracker.
 *          1) It first projects all map points into the image to find a potentially-visible-set (PVS);
 *          2) Then it tries to find some points of the PVS in the image;
 *          3) Then it updates camera pose according to any points found.
 *          4) Above may happen twice if a coarse tracking stage is performed.
 *          5) Finally it updates the tracker's current-frame-KeyFrame struct with any measurements made.
 */
void Tracker::TrackMap()
{
    // Some accounting which will be used for tracking quality assessment:
    for(int i=0; i<LEVELS; i++)
        manMeasAttempted[i] = manMeasFound[i] = 0;

    // The Potentially-Visible-Set (PVS) is split into pyramid levels.
    vector<TrackerData*> avPVS[LEVELS];
    for (auto &i : avPVS)
        i.reserve(500);

    // For all points in the map..
    for (auto &vpPoint : mMap.vpPoints) {
        MapPoint &p= *vpPoint;
        // Ensure that this map point has an associated TrackerData struct.
        if(!p.pTData)
            p.pTData = new TrackerData(&p);
        TrackerData &TData = *p.pTData;

        // Project according to current view, and if it's not in the image, skip.
        TData.Project(mse3CamFromWorld, mCamera);
        if(!TData.bInImage)
            continue;

        // Calculate camera projection derivatives of this point.
        TData.m2CamDerivs = mCamera.GetProjectionDerivs();

        // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
        TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
        if(TData.nSearchLevel == -1)
            continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

        // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
        TData.bSearched = false;
        TData.bFound = false;
        avPVS[TData.nSearchLevel].push_back(&TData);
    }

    // Next: A large degree of faffing about and deciding which points are going to be measured!
    // First, randomly shuffle the individual levels of the PVS.
    for (auto &i : avPVS)
        std::random_shuffle(i.begin(), i.end());

    // The next two data structs contain the list of points which will next
    // be searched for in the image, and then used in pose update.
    vector<TrackerData*> vNextToSearch;
    vector<TrackerData*> vIterationSet;

    // Tunable parameters to do with the coarse tracking stage:
    static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
    static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
    static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
    static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
    static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
    static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.

    unsigned int nCoarseMax = *gvnCoarseMax;
    unsigned int nCoarseRange = *gvnCoarseRange;

    mbDidCoarse = false;

    // Set of heuristics to check if we should do a coarse tracking stage.
    bool bTryCoarse = true;
    if(*gvnCoarseDisabled || mdMSDScaledVelocityMagnitude<*gvdCoarseMinVel || nCoarseMax==0) {
        bTryCoarse = false;
    }
    if(mbJustRecoveredSoUseCoarse) {
        bTryCoarse = true;
        nCoarseMax *= 2;
        nCoarseRange *= 2;
        mbJustRecoveredSoUseCoarse = false;
    }

    // If we do want to do a coarse stage, also check that there's enough high-level
    // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
    // with preference to LEVELS-1.
    if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin ) {
        // Now, fill the vNextToSearch struct with an appropriate number of
        // TrackerDatas corresponding to coarse map points! This depends on how many
        // there are in different pyramid levels compared to CoarseMin and CoarseMax.

        if (avPVS[LEVELS - 1].size() <= nCoarseMax) { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
            vNextToSearch = avPVS[LEVELS - 1];
            avPVS[LEVELS - 1].clear();
        } else { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
            for (unsigned int i = 0; i < nCoarseMax; i++)
                vNextToSearch.push_back(avPVS[LEVELS - 1][i]);
            avPVS[LEVELS - 1].erase(avPVS[LEVELS - 1].begin(), avPVS[LEVELS - 1].begin() + nCoarseMax);
        }

        // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
        if (vNextToSearch.size() < nCoarseMax) {
            unsigned int nMoreCoarseNeeded = nCoarseMax - (unsigned int)vNextToSearch.size();
            if (avPVS[LEVELS - 2].size() <= nMoreCoarseNeeded) {
                vNextToSearch = avPVS[LEVELS - 2];
                avPVS[LEVELS - 2].clear();
            } else {
                for (unsigned int i = 0; i < nMoreCoarseNeeded; i++)
                    vNextToSearch.push_back(avPVS[LEVELS - 2][i]);
                avPVS[LEVELS - 2].erase(avPVS[LEVELS - 2].begin(), avPVS[LEVELS - 2].begin() + nMoreCoarseNeeded);
            }
        }

        // Now go and attempt to find these points in the image!
        unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
        vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
        if (nFound >= *gvnCoarseMin) {
            mbDidCoarse = true;

            // If so: do ten Gauss-Newton pose updates iterations.
            for (int iter = 0; iter < 10; iter++) {
                if (iter != 0) { // Re-project the points on all but the first iteration.
                    for (auto &i : vIterationSet)
                        if (i->bFound)
                            i->ProjectAndDerivs(mse3CamFromWorld, mCamera);
                }
                for (auto &i : vIterationSet)
                    if (i->bFound)
                        i->CalcJacobian();

                // Hack: force the MEstimator to be pretty brutal with outliers beyond the fifth iteration.
                double dOverrideSigma = iter > 5 ? 1.0 : 0.0;

                // Calculate and apply the pose update...
                Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma);
                mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
            }
        }
    }

    // So, at this stage, we may or may not have done a coarse tracking stage.
    // Now do the fine tracking stage. This needs many more points!

    // Pixel search range for the fine stage.
    unsigned int nFineRange = mbDidCoarse ? 5 : 10;

    // What patches shall we use this time? The high-level ones are quite important, so do all of these, with sub-pixel refinement.
    {
        int l = LEVELS - 1;
        for (auto &i : avPVS[l])
            i->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        SearchForPoints(avPVS[l], nFineRange, 8);
        for (auto i : avPVS[l])
            vIterationSet.push_back(i);
    }

    {
        // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
        vNextToSearch.clear();
        for (int l = LEVELS - 2; l >= 0; l--)
            for (unsigned int i = 0; i < avPVS[l].size(); i++)
                vNextToSearch.push_back(avPVS[l][i]);

        // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit ourselves to 1000, and choose these randomly.
        static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
        int nFinePatchesToUse = *gvnMaxPatchesPerFrame - (int) vIterationSet.size();
        if (nFinePatchesToUse < 0)
            nFinePatchesToUse = 0;
        if ((int) vNextToSearch.size() > nFinePatchesToUse) {
            std::random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
            vNextToSearch.resize(static_cast<unsigned long>(nFinePatchesToUse)); // Chop!
        }

        if (mbDidCoarse) //If we did a coarse tracking stage: re-project and find derivs of fine points
            for (auto &i : vNextToSearch)
                i->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        SearchForPoints(vNextToSearch, nFineRange, 0); //Find fine points in image:
        for (auto i : vNextToSearch) //And attach them all to the end of the optimisation-set.
            vIterationSet.push_back(i);
    }

    // Again, ten gauss-newton pose update iterations.
    Vector<6> v6LastUpdate = Zeros;
    for(int iter = 0; iter<10; iter++) {
        // For a bit of time-saving: don't do full nonlinear reprojection at every iteration - it really isn't necessary!
        // Even this is probably overkill, the reason we do many iterations is for M-Estimator convergence rather than linearisation effects.
        bool bNonLinearIteration = iter == 0 || iter == 4 || iter == 9;
        if (iter != 0) { // Either way: first iteration doesn't need projection update.
            if (bNonLinearIteration) {
                for (auto &i : vIterationSet)
                    if (i->bFound)
                        i->ProjectAndDerivs(mse3CamFromWorld, mCamera);
            } else {
                for (auto &i : vIterationSet)
                    if (i->bFound)
                        i->LinearUpdate(v6LastUpdate);
            }
        }

        if (bNonLinearIteration)
            for (auto &i : vIterationSet)
                if (i->bFound)
                    i->CalcJacobian();

        // Again, an M-Estimator hack beyond the fifth iteration.
        double dOverrideSigma = iter > 5 ? 16.0 : 0.0;

        // Calculate and update pose; also store update vector for linear iteration updates.
        Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma, iter == 9);
        mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
        v6LastUpdate = v6Update;
    }

    if(mbDraw) {
        glPointSize(6);
        glEnable(GL_BLEND);
        glEnable(GL_POINT_SMOOTH);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_POINTS);
        for (auto it = vIterationSet.rbegin(); it != vIterationSet.rend(); it++) {
            if (!(*it)->bFound)
                continue;
            glColor(Level::mvLevelColors[(*it)->nSearchLevel]);
            glVertex((*it)->v2Image);
        }
        glEnd();
        glDisable(GL_BLEND);
    }

    // Update the current keyframe with info on what was found in the frame.
    // Strictly speaking this is unnecessary to do every frame, it'll only be
    // needed if the KF gets added to MapMaker. Do it anyway.
    // Export pose to current keyframe:
    mCurrentKF.se3CfromW = mse3CamFromWorld;

    // Record successful measurements. Use the KeyFrame-Measurement struct for this.
    mCurrentKF.mMeasurements.clear();
    for (auto &it : vIterationSet) {
        if (!it->bFound)
            continue;
        Measurement m;
        m.v2RootPos = it->v2Found;
        m.nLevel = it->nSearchLevel;
        m.bSubPix = it->bDidSubPix;
        mCurrentKF.mMeasurements[&(it->Point)] = m;
    }

    // Finally, find the mean scene depth from tracked features
    {
        double dSum = 0;
        double dSumSq = 0;
        int nNum = 0;
        for (auto &it : vIterationSet) {
            if (it->bFound) {
                double z = it->v3Cam[2];
                dSum += z;
                dSumSq += z * z;
                nNum++;
            }
        }
        if(nNum > 20)
        {
            mCurrentKF.dSceneDepthMean = dSum/nNum;
            mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
        }
    }
}

void Tracker::TrackMapLocal() {

    set<MapPoint*> sMapPoints;
    const int nKF = 4;
    const int nPt = 200;

    if(mMap.vpKeyFrames.size() < nKF+1) {
        for(int i=0; i<mMap.vpPoints.size() && i<nPt; ++i){
            sMapPoints.insert(mMap.vpPoints[i]);
        }
    }
    else {
        set<KeyFrame *> sAdjustSet;
        KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
        sAdjustSet.insert(pkfNewest);

        vector<KeyFrame *> vClosest = mMapMaker.NClosestKeyFrames(*pkfNewest, nKF);
        for (int i = 0; i < nKF; i++)
            sAdjustSet.insert(vClosest[i]);

        int nMapPoints = 0;
        for (auto iter : sAdjustSet) {
            map<MapPoint *, Measurement> &mKFMeas = iter->mMeasurements;
            for (auto &mKFMea : mKFMeas) {
                sMapPoints.insert(mKFMea.first);
                nMapPoints++;
                if(nMapPoints>nPt)
                    break;
            }
        }
    }

    for(int i=0; i<LEVELS; i++)
        manMeasAttempted[i] = manMeasFound[i] = 0;

    vector<TrackerData*> avPVS[LEVELS];
    for (auto &i : avPVS)
        i.reserve(100);

    for (auto &spPoint : sMapPoints) {
        MapPoint &p= *spPoint;

        if(!p.pTData)
            p.pTData = new TrackerData(&p);
        TrackerData &TData = *p.pTData;

        TData.Project(mse3CamFromWorld, mCamera);
        if(!TData.bInImage)
            continue;

        TData.m2CamDerivs = mCamera.GetProjectionDerivs();

        TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
        if(TData.nSearchLevel == -1)
            continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

        TData.bSearched = false;
        TData.bFound = false;
        avPVS[TData.nSearchLevel].push_back(&TData);
    }

    for (auto &i : avPVS)
        std::random_shuffle(i.begin(), i.end());


    vector<TrackerData*> vIterationSet;

    for (int l = LEVELS - 1; l >= 0; l--) {
        for (auto &i : avPVS[l])
            i->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        SearchForPoints(avPVS[l], 20, 8);
        for (auto i : avPVS[l])
            vIterationSet.push_back(i);
    }

    // Again, ten gauss-newton pose update iterations.
    Vector<6> v6LastUpdate = Zeros;
    for(int iter = 0; iter<10; iter++) {
        // For a bit of time-saving: don't do full nonlinear reprojection at every iteration - it really isn't necessary!
        // Even this is probably overkill, the reason we do many iterations is for M-Estimator convergence rather than linearisation effects.
        bool bNonLinearIteration = iter == 0 || iter == 4 || iter == 9;
        if (iter != 0) { // Either way: first iteration doesn't need projection update.
            if (bNonLinearIteration) {
                for (auto &i : vIterationSet)
                    if (i->bFound)
                        i->ProjectAndDerivs(mse3CamFromWorld, mCamera);
            } else {
                for (auto &i : vIterationSet)
                    if (i->bFound)
                        i->LinearUpdate(v6LastUpdate);
            }
        }

        if (bNonLinearIteration)
            for (auto &i : vIterationSet)
                if (i->bFound)
                    i->CalcJacobian();

        // Again, an M-Estimator hack beyond the fifth iteration.
        double dOverrideSigma = iter > 5 ? 16.0 : 0.0;

        // Calculate and update pose; also store update vector for linear iteration updates.
        Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma, iter == 9);
        mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
        v6LastUpdate = v6Update;
    }

    if(mbDraw) {
        glPointSize(6);
        glEnable(GL_BLEND);
        glEnable(GL_POINT_SMOOTH);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_POINTS);
        for (auto it = vIterationSet.rbegin(); it != vIterationSet.rend(); it++) {
            if (!(*it)->bFound)
                continue;
            glColor(Level::mvLevelColors[(*it)->nSearchLevel]);
            glVertex((*it)->v2Image);
        }
        glEnd();
        glDisable(GL_BLEND);
    }

    mCurrentKF.se3CfromW = mse3CamFromWorld;

    // Record successful measurements. Use the KeyFrame-Measurement struct for this.
    mCurrentKF.mMeasurements.clear();
    for (auto &it : vIterationSet) {
        if (!it->bFound)
            continue;
        Measurement m;
        m.v2RootPos = it->v2Found;
        m.nLevel = it->nSearchLevel;
        m.bSubPix = it->bDidSubPix;
        mCurrentKF.mMeasurements[&(it->Point)] = m;
    }

    // Finally, find the mean scene depth from tracked features
    {
        double dSum = 0;
        double dSumSq = 0;
        int nNum = 0;
        for (auto &it : vIterationSet) {
            if (it->bFound) {
                double z = it->v3Cam[2];
                dSum += z;
                dSumSq += z * z;
                nNum++;
            }
        }
        if(nNum > 20)
        {
            mCurrentKF.dSceneDepthMean = dSum/nNum;
            mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
        }
    }

}


/**
 * @brief Find points in the image
 * @param vTD TrackerData vector
 * @param nRange pixel search radius
 * @param nSubPixIts sub-pixel iterations
 * @return
 */
unsigned int Tracker::SearchForPoints(vector<TrackerData*> &vTD, unsigned int nRange, int nSubPixIts)
{
    unsigned int nFound = 0;
    for (auto &i : vTD) {
        TrackerData &TD = *i;
        PatchFinder &Finder = TD.Finder;
        Finder.MakeTemplateCoarseCont(TD.Point);
        if (Finder.TemplateBad()) {
            TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
            continue;
        }

        manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta

        bool bFound = Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange);
        TD.bSearched = true;
        if (!bFound) {
            TD.bFound = false;
            continue;
        }

        TD.bFound = true;
        TD.dSqrtInvNoise = (1.0 / Level::LevelScale(Finder.GetLevel()));

        nFound++;
        manMeasFound[Finder.GetLevel()]++;

        // Found the patch in coarse search - are Sub-pixel iterations wanted too?
        if (nSubPixIts > 0) {
            TD.bDidSubPix = true;
            Finder.MakeSubPixTemplate();
            bool bSubPixConverges = Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);
            if (!bSubPixConverges) { // If subpix doesn't converge, the patch location is probably very dubious!
                TD.bFound = false;
                nFound--;
                manMeasFound[Finder.GetLevel()]--;
                continue;
            }
            TD.v2Found = Finder.GetSubPixPos();
        } else {
            TD.v2Found = Finder.GetCoarsePosAsVector();
            TD.bDidSubPix = false;
        }
    }
    return nFound;
}

/**
 * @brief Calculate a pose update 6-vector from a bunch of image measurements.
 * @param vTD
 * @param dOverrideSigma
 * @param bMarkOutliers set to true records any instances of a point being marked an outlier measurement by the Tukey MEstimator.
 * @return a pose update
 * @details
 * @verbatim
 * 1) User-selectable M-Estimator.
 * 2) Normally this robustly estimates a sigma-squared for all the measurements
 *    to reduce outlier influence, but this can be overridden
 *    if dOverrideSigma is positive.
 * @endverbatim
 */
Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
    int nEstimator = 0;
    static gvar3<string> gvsEstimator("Tracker.MEstimator", "Tukey", SILENT);
    if(*gvsEstimator == "Tukey")
        nEstimator = 0;
    else if(*gvsEstimator == "Cauchy")
        nEstimator = 1;
    else if(*gvsEstimator == "Huber")
        nEstimator = 2;
    else
    {
        cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
        nEstimator = 0;
        *gvsEstimator = "Tukey";
    }

    // Find the covariance-scaled reprojection error for each measurement.
    // Also, store the square of these quantities for M-Estimator sigma squared estimation.
    vector<double> vdErrorSquared;
    for (auto &f : vTD) {
        TrackerData &TD = *f;
        if(!TD.bFound)
            continue;
        TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
        vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
    }
    if(vdErrorSquared.empty())
        return makeVector( 0,0,0,0,0,0);

    // What is the distribution of errors?
    double dSigmaSquared;
    if(dOverrideSigma > 0)
        dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
    else
    {
        if (nEstimator == 0)
            dSigmaSquared = cg::Tukey::FindSigmaSquared(vdErrorSquared);
        else if(nEstimator == 1)
            dSigmaSquared = cg::Cauchy::FindSigmaSquared(vdErrorSquared);
        else
            dSigmaSquared = cg::Huber::FindSigmaSquared(vdErrorSquared);
    }

    // The TooN WLSCholesky class handles reweighted least squares. It just needs errors and jacobians.
    WLS<6> wls;
    wls.add_prior(100.0); // Stabilising prior
    for (auto &f : vTD) {
        TrackerData &TD = *f;
        if(!TD.bFound)
            continue;
        Vector<2> &v2 = TD.v2Error_CovScaled;

        double dErrorSq = v2 * v2;
        double dWeight;
        if(nEstimator == 0)
            dWeight= cg::Tukey::Weight(dErrorSq, dSigmaSquared);
        else if(nEstimator == 1)
            dWeight= cg::Cauchy::Weight(dErrorSq, dSigmaSquared);
        else
            dWeight= cg::Huber::Weight(dErrorSq, dSigmaSquared);
        // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
        if(dWeight == 0.0) {
            if (bMarkOutliers)
                TD.Point.nMEstimatorOutlierCount++;
            continue;
        }
        else
            if(bMarkOutliers)
                TD.Point.nMEstimatorInlierCount++;

        Matrix<2,6> &m26Jac = TD.m26Jacobian;
        wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
        wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
    wls.compute();
    return wls.get_mu();
}

/**
 * @brief Just add the current velocity to the current pose.
 *        N.b. this doesn't actually use time in any way,i.e. it assumes a one-frame-per-second camera.
 *        Skipped frames etc are not handled properly here.
 */
void Tracker::PredictPoseWithMotionModel()
{
    mse3StartPos = mse3CamFromWorld;
    Vector<6> v6Velocity = mv6CameraVelocity;
    if(mbUseSBIInit)
    {
        mpSBILastFrame->MakeJacs(mpSBILastFrame->mimTemplate, mpSBILastFrame->mimImageJacs);

        std::pair<SE3<>, double> result_pair = mpSBIThisFrame->CalcSBIRotation(mpSBILastFrame, mCamera);

        Vector<6> v6SBIRot = result_pair.first.ln();

        v6Velocity.slice<3,3>() = v6SBIRot.slice<3,3>();
        v6Velocity[0] = 0.0;
        v6Velocity[1] = 0.0;
    }
    mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
}

/**
 * @brief The motion model is entirely the tracker's, and is kept as
 *        a decaying constant velocity model.
 */
void Tracker::UpdateMotionModel()
{
    SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
    Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);

    static gvar3<int> gvnConstVel("Tracker.UseConstantVelocity", 1, SILENT);
    auto bConstVel = (bool)*gvnConstVel;
    if(bConstVel) {
        mv6CameraVelocity = v6Motion;
    }
    else {
        Vector<6> v6OldVel = mv6CameraVelocity;
        mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
    }

    // Also make an estimate of this which has been scaled by the mean scene depth.
    // This is used to decide if we should use a coarse tracking stage.
    // We can tolerate more translational vel when far away from scene!
    Vector<6> v6 = mv6CameraVelocity;
    v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
    mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

/**
 * @brief Some heuristics to decide if tracking is any good, for this frame.
 * @details This influences decisions to add key-frames, and eventually causes the tracker to attempt relocalisation.
 */
void Tracker::AssessTrackingQuality()
{
    int nTotalAttempted = 0;
    int nTotalFound = 0;
    int nLargeAttempted = 0;
    int nLargeFound = 0;

    for(int i=0; i<LEVELS; i++) {
        nTotalAttempted += manMeasAttempted[i];
        nTotalFound += manMeasFound[i];
        if (i >= 2) {
            nLargeAttempted += manMeasAttempted[i];
            nLargeFound += manMeasFound[i];
        }
    }

    if(nTotalFound == 0 || nTotalAttempted == 0)
        mTrackingQuality = BAD;
    else {
        double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
        double dLargeFracFound;
        if (nLargeAttempted > 10)
            dLargeFracFound = (double) nLargeFound / nLargeAttempted;
        else
            dLargeFracFound = dTotalFracFound;

        static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
        static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);

        if (dTotalFracFound > *gvdQualityGood)
            mTrackingQuality = GOOD;
        else if (dLargeFracFound < *gvdQualityLost)
            mTrackingQuality = BAD;
        else { // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)
            KeyFrame *pClosest = mMapMaker.ClosestKeyFrame(mCurrentKF);
            double dDist = mMapMaker.KeyFrameLinearDist(mCurrentKF, *pClosest);
            if (dDist > mMapMaker.GetWiggleScale() * 10.0)
                mTrackingQuality = BAD;
        }
    }

    if(mTrackingQuality==BAD)
        mnLostFrames++;
    else
        mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
    return mMessageForUser.str();
}

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here








