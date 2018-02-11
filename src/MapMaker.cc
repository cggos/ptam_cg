// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#include <fstream>
#include <algorithm>
#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>
#include <gvars3/instances.h>

#include "Bundle.h"
#include "PatchFinder.h"
#include "HomographyInit.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, const ATANCamera &cam)
    : mMap(m), mCamera(cam)
{
    mbResetRequested = false;
    Reset();
    start(); // This CVD::thread func starts the map-maker thread with function run()
    GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
    GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.1, SILENT); // Default to 10cm between keyframes
}

void MapMaker::Reset()
{
    // This is only called from within the mapmaker thread...
    mMap.Reset();
    mvFailureQueue.clear();
    while(!mqNewQueue.empty()) mqNewQueue.pop();
    mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
    mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes
    mbBundleRunning = false;
    mbBundleConverged_Full = true;
    mbBundleConverged_Recent = true;
    mbResetDone = true;
    mbResetRequested = false;
    mbBundleAbortRequested = false;
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run()
{

#ifdef WIN32
    // For some reason, I get tracker thread starvation on Win32 when
    // adding key-frames. Perhaps this will help:
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

    while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
    {
        CHECK_RESET;
        sleep(5); // Sleep not really necessary, especially if mapmaker is busy
        CHECK_RESET;

        // Handle any GUI commands encountered..
        while(!mvQueuedCommands.empty())
        {
            GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
            mvQueuedCommands.erase(mvQueuedCommands.begin());
        }

        if(!mMap.IsGood())  // Nothing to do if there is no map yet!
            continue;

        // From here on, mapmaker does various map-maintenance jobs in a certain priority
        // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
        // then that takes high priority.

        CHECK_RESET;
        // Should we run local bundle adjustment?
        if(!mbBundleConverged_Recent && QueueSize() == 0)
            BundleAdjustRecent();

        CHECK_RESET;
        // Are there any newly-made map points which need more measurements from older key-frames?
        if(mbBundleConverged_Recent && QueueSize() == 0)
            ReFindNewlyMade();

        CHECK_RESET;
        // Run global bundle adjustment?
        if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
            BundleAdjustAll();

        CHECK_RESET;
        // Very low priorty: re-find measurements marked as outliers
        if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
            ReFindFromFailureQueue();

        CHECK_RESET;
        HandleBadPoints();

        CHECK_RESET;
        // Any new key-frames to be added?
        if(QueueSize() > 0)
            AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
    }
}

void MapMaker::RequestReset()
{
    mbResetDone = false;
    mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
    return mbResetDone;
}

/**
 * @brief Does some heuristic checks on all points in the map to see if
 *        they should be flagged as bad, based on tracker feedback.
 */
void MapMaker::HandleBadPoints()
{
    // Did the tracker see this point as an outlier more often than as an inlier?
    for (auto &vpPoint : mMap.vpPoints) {
        MapPoint &p = *vpPoint;
        if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount)
            p.bBad = true;
    }

    // All points marked as bad will be erased - erase all records of them
    // from keyframes in which they might have been measured.
    for (auto &vpPoint : mMap.vpPoints) {
        if (vpPoint->bBad) {
            MapPoint *p = vpPoint;
            for (auto &vpKeyFrame : mMap.vpKeyFrames) {
                KeyFrame &k = *vpKeyFrame;
                if (k.mMeasurements.count(p))
                    k.mMeasurements.erase(p);
            }
        }
    }
    mMap.MoveBadPointsToTrash();
}

MapMaker::~MapMaker()
{
    mbBundleAbortRequested = true;
    stop(); // makes shouldStop() return true
    cout << "Waiting for mapmaker to die.." << endl;
    join();
    cout << " .. mapmaker has died." << endl;
}

/**
 * @brief Finds 3d coords of point in reference frame B from two z=1 plane projections
 * @param se3AfromB RT from B to A
 * @param v2A coordinates in z=1 plane from A
 * @param v2B coordinates in z=1 plane from B
 * @return 3D coordinate of point in reference frame B
 */
Vector<3> MapMaker::Triangulate(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
    Matrix<3,4> PDash;
    PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
    PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

    Matrix<4> A;
    A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
    A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
    A[2] = v2A[0] * PDash[2] - PDash[0];
    A[3] = v2A[1] * PDash[2] - PDash[1];

    SVD<4,4> svd(A);
    Vector<4> v4Smallest = svd.get_VT()[3];
    if(v4Smallest[3] == 0.0)
        v4Smallest[3] = 0.00001;
    return project(v4Smallest);
}

/**
 * @brief Be similiar to MapMaker::Triangulate() solving overdetermined equation
 *        Finds 3d coords of point in reference frame B from two z=1 plane projections
 * @param se3AfromB RT from B to A
 * @param v2A coordinates in z=1 plane from A
 * @param v2B coordinates in z=1 plane from B
 * @return 3D coordinate of point in reference frame B
 * @details the algorithm:\n
 *          (1) With \f$ O_B, P_B, P_W \f$ and \f$ O_A, P_A, P_W \f$ collinear respectively,we get
 *              \f[
 *                 \begin{cases}
 *                    O_BP_B \times P_W = 0 \\
 *                    O_AP_A \times T_{AB}P_W = 0
 *                 \end{cases}
 *              \f]
 *          (2) Transform above equations set to Matrixs
 *              \f[
 *                 A = \begin{bmatrix} P_B \times I \\ P_A \times T_{AB} \end{bmatrix},
 *                 A \cdot P_W = 0
 *              \f]
 *          (3) Get the result from the sigular column vector respond to the smallest singular value of SVD(A)
 *              \f[
 *                 SVD(A) = U \Sigma V^T,
 *                 P_W = V[3]
 *              \f]
 */
Vector<3> MapMaker::TriangulateNew(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
    Vector<3> v3A = unproject(v2A);
    Vector<3> v3B = unproject(v2B);

    Matrix<3> m3A = TooN::Zeros;
    m3A[0][1] = -v3A[2];
    m3A[0][2] =  v3A[1];
    m3A[1][2] = -v3A[0];
    m3A[1][0] = -m3A[0][1];
    m3A[2][0] = -m3A[0][2];
    m3A[2][1] = -m3A[1][2];
    Matrix<3> m3B = TooN::Zeros;
    m3B[0][1] = -v3B[2];
    m3B[0][2] =  v3B[1];
    m3B[1][2] = -v3B[0];
    m3B[1][0] = -m3B[0][1];
    m3B[2][0] = -m3B[0][2];
    m3B[2][1] = -m3B[1][2];

    Matrix<3,4> m34AB;
    m34AB.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
    m34AB.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

    SE3<> se3I;
    Matrix<3,4> m34I;
    m34I.slice<0,0,3,3>() = se3I.get_rotation().get_matrix();
    m34I.slice<0,3,3,1>() = se3I.get_translation().as_col();

    Matrix<3,4> PDashA = m3A * m34AB;
    Matrix<3,4> PDashB = m3B * m34I;

    Matrix<6,4> A;
    A.slice<0,0,3,4>() = PDashA;
    A.slice<3,0,3,4>() = PDashB;

    SVD<6,4> svd(A);
    Vector<4> v4Smallest = svd.get_VT()[3];
    if(v4Smallest[3] == 0.0)
        v4Smallest[3] = 0.00001;

    return project(v4Smallest);
}

/**
 * @brief generates the initial MapPoints and TrackerPose from two keyframes and a vector of image correspondences
 * @param kF first KeyFrame
 * @param kS second KeyFrame
 * @param vTrailMatches initial matches from two keyframes
 * @param se3TrackerPose out tracker pose
 * @return true or false
 */
bool MapMaker::InitFromStereo(KeyFrame &kF, KeyFrame &kS, vector<pair<ImageRef, ImageRef> > &vTrailMatches, SE3<> &se3TrackerPose)
{
    mCamera.SetImageSize(kF.aLevels[0].im.size());

    vector<HomographyMatch> vMatches;
    for (auto &vTrailMatche : vTrailMatches) {
        HomographyMatch m;
        m.v2CamPlaneFirst = mCamera.UnProject(vTrailMatche.first);
        m.v2CamPlaneSecond = mCamera.UnProject(vTrailMatche.second);
        m.m2PixelProjectionJac = mCamera.GetProjectionDerivs();
        vMatches.push_back(m);
    }

    SE3<> se3;
    HomographyInit HomographyInit;
    bool bGood = HomographyInit.Compute(vMatches, 5.0, se3);
    if(!bGood) {
        cout << "  Could not init from stereo pair, try again." << endl;
        return false;
    }

    // Check that the initialiser estimated a non-zero baseline
    double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
    if(dTransMagn < 1e-6) {
        cout << "  Estimated zero baseline from stereo pair, try again." << endl;
        return false;
    }
    // change the scale of the map so the second camera is wiggleScale away from the first
    mdWiggleScale = *mgvdWiggleScale;
    se3.get_translation() *= mdWiggleScale/dTransMagn;

    auto *pkFirst = new KeyFrame();
    auto *pkSecond = new KeyFrame();
    *pkFirst = kF;
    *pkSecond = kS;

    pkFirst->bFixed = true;
    pkFirst->se3CfromW = SE3<>();

    pkSecond->bFixed = false;
    pkSecond->se3CfromW = se3;

    // Construct map from the stereo matches.
    PatchFinder finder;
    for(unsigned int i=0; i<vMatches.size(); i++)
    {
        auto *p = new MapPoint();

        // Patch source stuff:
        p->pPatchSourceKF = pkFirst;
        p->nSourceLevel = 0;
        p->v3Normal_NC = makeVector( 0,0,-1);
        p->irCenter = vTrailMatches[i].first;
        p->v3Center_NC = unproject(mCamera.UnProject(p->irCenter));
        p->v3OneDownFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(0,1)));
        p->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(1,0)));
        normalize(p->v3Center_NC);
        normalize(p->v3OneDownFromCenter_NC);
        normalize(p->v3OneRightFromCenter_NC);
        p->RefreshPixelVectors();

        // Do sub-pixel alignment on the second image
        finder.MakeTemplateCoarseNoWarp(*p->pPatchSourceKF,p->nSourceLevel,p->irCenter);
        finder.MakeSubPixTemplate();
        finder.SetSubPixPos(vec(vTrailMatches[i].second));
        bool bGood = finder.IterateSubPixToConvergence(*pkSecond,10);
        if(!bGood) {
            delete p;
            continue;
        }

        Vector<2> v2SecondPos = finder.GetSubPixPos();
        Vector<2> v2CamPlaneSecond = mCamera.UnProject(v2SecondPos);
        //Triangulate point
        p->v3WorldPos = pkFirst->se3CfromW.inverse() * Triangulate(se3, v2CamPlaneSecond, vMatches[i].v2CamPlaneFirst);
        if(p->v3WorldPos[2] < 0.0) { //check if the point behind the map
            delete p;
            continue;
        }

        p->pMMData = new MapMakerData();
        mMap.vpPoints.push_back(p);

        // Construct first two measurements and insert into relevant DBs:
        Measurement mFirst;
        mFirst.nLevel = 0;
        mFirst.Source = Measurement::SRC_ROOT;
        mFirst.v2RootPos = vec(vTrailMatches[i].first);
        mFirst.bSubPix = true;
        pkFirst->mMeasurements[p] = mFirst;
        p->pMMData->sMeasurementKFs.insert(pkFirst);

        Measurement mSecond;
        mSecond.nLevel = 0;
        mSecond.Source = Measurement::SRC_TRAIL;
        mSecond.v2RootPos = finder.GetSubPixPos();
        mSecond.bSubPix = true;
        pkSecond->mMeasurements[p] = mSecond;
        p->pMMData->sMeasurementKFs.insert(pkSecond);
    }

    mMap.vpKeyFrames.push_back(pkFirst);
    mMap.vpKeyFrames.push_back(pkSecond);
    pkFirst->MakeKeyFrame_Rest();
    pkSecond->MakeKeyFrame_Rest();

    for(int i=0; i<5; i++)
        BundleAdjustAll();

    // Estimate the feature depth distribution in the first two key-frames (Needed for epipolar search)
    RefreshSceneDepth(pkFirst);
    RefreshSceneDepth(pkSecond);
    mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

    AddSomeMapPoints(0);
    AddSomeMapPoints(3);
    AddSomeMapPoints(1);
    AddSomeMapPoints(2);

    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;

    while(!mbBundleConverged_Full) {
        BundleAdjustAll();
        if (mbResetRequested)
            return false;
    }

    // Rotate and translate the map so the dominant plane is at z=0:
    ApplyGlobalTransformationToMap(CalcPlaneAligner());

    mMap.bGood = true;
    se3TrackerPose = pkSecond->se3CfromW;

    cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;

    return true;
}

/**
 * @brief Thins out a key-frame's candidate list.
 * @param k
 * @param nLevel
 * @details Candidates are those salient corners where the mapmaker will attempt to make a new map point by epipolar search.
 *          We don't want to make new points where there are already existing map points, this routine erases such candidates.
 *          Operates on a single level of a keyframe.
 */
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel)
{
    vector<ImageRef> irBusyLevelPos;
    // Make a list of `busy' image locations, which already have features at the same level or at one level higher.
    for (auto &mMeasurement : k.mMeasurements) {
        if (mMeasurement.second.nLevel == nLevel || mMeasurement.second.nLevel == nLevel + 1)
            irBusyLevelPos.push_back(ir_rounded(mMeasurement.second.v2RootPos / Level::LevelScale(nLevel)));
    }

    // Only keep those candidates further than 10 pixels away from busy positions.
    vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
    vector<Candidate> vCGood;
    unsigned int nMinMagSquared = 10*10;
    for (auto &i : vCSrc) {
        ImageRef irC = i.irLevelPos;
        bool bGood = true;
        for (auto irB : irBusyLevelPos) {
            if ((irB - irC).mag_squared() < nMinMagSquared) {
                bGood = false;
                break;
            }
        }
        if (bGood)
            vCGood.push_back(i);
    }
    vCSrc = vCGood;
}

/**
 * @brief Adds map points by epipolar search to the last-added key-frame, at a single specified pyramid level.
 *        Does epipolar search in the target keyframe as closest by the ClosestKeyFrame function.
 * @param nLevel
 */
void MapMaker::AddSomeMapPoints(int nLevel)
{
    KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
    KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));

    ThinCandidates(kSrc, nLevel);

    for(unsigned int i = 0; i<kSrc.aLevels[nLevel].vCandidates.size(); i++)
        AddPointEpipolar(kSrc, kTarget, nLevel, i);
}

/**
 * @brief Rotates/translates the whole map and all keyframes
 * @param se3NewFromOld the RT from Old to New
 */
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
    for (auto &vpKeyFrame : mMap.vpKeyFrames)
        vpKeyFrame->se3CfromW = vpKeyFrame->se3CfromW * se3NewFromOld.inverse();

    for (auto &vpPoint : mMap.vpPoints) {
        vpPoint->v3WorldPos = se3NewFromOld * vpPoint->v3WorldPos;
        vpPoint->RefreshPixelVectors();
    }
}

/**
 * @brief The tracker entry point for adding a new keyframe
 * @param k KeyFrame to be added
 * @details the tracker thread doesn't want to hang about, so just dumps it on the top of the mapmaker's queue
 *          to be dealt with later, and return.
 */
void MapMaker::AddKeyFrame(KeyFrame &k)
{
    auto *pK = new KeyFrame;
    *pK = k;
    pK->pSBI = nullptr; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
    mvpKeyFrameQueue.push_back(pK);
    if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
        mbBundleAbortRequested = true;
}

/**
 * @brief Mapmaker's code to handle incoming key-frames.
 */
void MapMaker::AddKeyFrameFromTopOfQueue()
{
    if(mvpKeyFrameQueue.empty())
        return;

    KeyFrame *pK = mvpKeyFrameQueue[0];
    mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
    pK->MakeKeyFrame_Rest();
    mMap.vpKeyFrames.push_back(pK);
    // Any measurements? Update the relevant point's measurement counter status map
    for(auto it = pK->mMeasurements.begin(); it!=pK->mMeasurements.end(); it++) {
        it->first->pMMData->sMeasurementKFs.insert(pK);
        it->second.Source = Measurement::SRC_TRACKER;
    }

    // And maybe we missed some - this now adds to the map itself, too.
    ReFindInSingleKeyFrame(*pK);

    AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
    AddSomeMapPoints(0);
    AddSomeMapPoints(1);
    AddSomeMapPoints(2);

    mbBundleConverged_Full = false;
    mbBundleConverged_Recent = false;
}

/**
 * @brief Tries to make a new map point out of a single candidate point by searching for that point in another keyframe,
 *        and triangulating if a match is found.
 * @param kSrc source KeyFrame
 * @param kTarget target KeyFrame
 * @param nLevel level of candidate in the source KeyFrame
 * @param nCandidate index of Candidate
 * @return true or false
 */
bool MapMaker::AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate)
{
    static Image<Vector<2> > imUnProj;
    static bool bMadeCache = false;
    if(!bMadeCache) {
        imUnProj.resize(kSrc.aLevels[0].im.size());
        ImageRef ir;
        do imUnProj[ir] = mCamera.UnProject(ir);
        while (ir.next(imUnProj.size()));
        bMadeCache = true;
    }

    int nLevelScale = Level::LevelScale(nLevel);

    Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];
    ImageRef irLevelPos = candidate.irLevelPos;
    Vector<2> v2RootPos = Level::LevelZeroPos(irLevelPos, nLevel);

    //the source candidate point in z=1 plane in source coord, we express it as 'P1'
    Vector<3> v3Ray_SC = unproject(mCamera.UnProject(v2RootPos));
    normalize(v3Ray_SC); //unit vector
    //unit vector in world coord  whose direction is same to the vector 'OP1' in source coord
    Vector<3> v3RayUnit_WC  = kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC;
    //unit vector in target coord whose direction is same to the vector 'OP1' in source coord
    Vector<3> v3LineDirn_TC = kTarget.se3CfromW.get_rotation() * v3RayUnit_WC;

    // Restrict epipolar search to a relatively narrow depth range to increase reliability
    double dMean  = kSrc.dSceneDepthMean;
    double dSigma = kSrc.dSceneDepthSigma;
    double dStartDepth = max(mdWiggleScale, dMean - dSigma);
    double dEndDepth   = min(40 * mdWiggleScale, dMean + dSigma);

    Vector<3> v3CamCenter_WC = kSrc.se3CfromW.inverse().get_translation();  //the source camera center in world coord
    Vector<3> v3CamCenter_TC = kTarget.se3CfromW * v3CamCenter_WC;          //the source camera center in target coord
    Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC; //the far-away end
    Vector<3> v3RayEnd_TC   = v3CamCenter_TC + dEndDepth   * v3LineDirn_TC; //the far-away end

    // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
    if(v3RayEnd_TC[2] <= v3RayStart_TC[2])
        return false;
    if(v3RayEnd_TC[2] <= 0.0 )
        return false;
    if(v3RayStart_TC[2] <= 0.0)
        v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);

    Vector<2> v2A = project(v3RayStart_TC); //one     end of the epipolar line in z=1 plane in target coord
    Vector<2> v2B = project(v3RayEnd_TC);   //another end of the epipolar line in z=1 plane in target coord
    Vector<2> v2AlongProjectedLine = v2A-v2B;

    if(v2AlongProjectedLine * v2AlongProjectedLine < 1e-8) {
        cout << "v2AlongProjectedLine too small." << endl;
        return false;
    }
    normalize(v2AlongProjectedLine);

    Vector<2> v2Normal;
    v2Normal[0] = v2AlongProjectedLine[1];
    v2Normal[1] = -v2AlongProjectedLine[0];

    //distance from v2AlongProjectedLine to the center or origin in z=1 plane in target coord
    double dNormDist = v2A * v2Normal;
    if(fabs(dNormDist) > mCamera.LargestRadiusInImage() )
        return false;

    double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
    double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
    if(dMinLen < -2.0)  dMinLen = -2.0;
    if(dMaxLen < -2.0)  dMaxLen = -2.0;
    if(dMinLen > 2.0)   dMinLen = 2.0;
    if(dMaxLen > 2.0)   dMaxLen = 2.0;

    // Find current-frame corners which might match this
    PatchFinder Finder;
    Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
    if(Finder.TemplateBad())
        return false;

    //look up table (all over corners in z=1 plane in target coord)
    vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
    vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
    if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
    {
        for(unsigned int i=0; i<vIR.size(); i++)
            vv2Corners.push_back(imUnProj[ir(Level::LevelZeroPos(vIR[i], nLevel))]);
        kTarget.aLevels[nLevel].bImplaneCornersCached = true;
    }

    int nBest = -1;
    int nBestZMSSD = Finder.mnMaxSSD + 1;
    double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
    double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;
    for(unsigned int i=0; i<vv2Corners.size(); i++)
    {
        Vector<2> v2Im = vv2Corners[i];
        double dDistDiff = dNormDist - v2Im * v2Normal;
        if(dDistDiff * dDistDiff > dMaxDistSq)    //skip if not along epi line
            continue;
        if(v2Im * v2AlongProjectedLine < dMinLen) //skip if not far enough along line
            continue;
        if(v2Im * v2AlongProjectedLine > dMaxLen) //skip if too far
            continue;
        int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
        if(nZMSSD < nBestZMSSD) {
            nBest = i;
            nBestZMSSD = nZMSSD;
        }
    }
    if(nBest == -1)
        return false;

    //  Found a likely candidate along epipolar ray
    Finder.MakeSubPixTemplate();
    Finder.SetSubPixPos(Level::LevelZeroPos(vIR[nBest], nLevel));
    bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);
    if(!bSubPixConverges)
        return false;
    Vector<2> v2SubPosTarget = Finder.GetSubPixPos();

    // Now triangulate the 3d point...
    SE3<> se3SrcfromTarget = kSrc.se3CfromW * kTarget.se3CfromW.inverse();
    Vector<3> v3New = kTarget.se3CfromW.inverse() * Triangulate(se3SrcfromTarget, mCamera.UnProject(v2RootPos), mCamera.UnProject(v2SubPosTarget));

    MapPoint *pNew = new MapPoint;
    pNew->v3WorldPos = v3New;
    pNew->pMMData = new MapMakerData();

    // Patch source stuff:
    pNew->pPatchSourceKF = &kSrc;
    pNew->nSourceLevel = nLevel;
    pNew->v3Normal_NC = makeVector( 0,0,-1);
    pNew->irCenter = irLevelPos;
    pNew->v3Center_NC = unproject(mCamera.UnProject(v2RootPos));
    pNew->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
    pNew->v3OneDownFromCenter_NC  = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

    normalize(pNew->v3Center_NC);
    normalize(pNew->v3OneDownFromCenter_NC);
    normalize(pNew->v3OneRightFromCenter_NC);

    pNew->RefreshPixelVectors();
    
    mMap.vpPoints.push_back(pNew);
    mqNewQueue.push(pNew);

    Measurement m;
    m.Source = Measurement::SRC_ROOT;
    m.v2RootPos = v2RootPos;
    m.nLevel = nLevel;
    m.bSubPix = true;
    kSrc.mMeasurements[pNew] = m;

    m.Source = Measurement::SRC_EPIPOLAR;
    m.v2RootPos = Finder.GetSubPixPos();
    kTarget.mMeasurements[pNew] = m;

    pNew->pMMData->sMeasurementKFs.insert(&kSrc);
    pNew->pMMData->sMeasurementKFs.insert(&kTarget);

    return true;
}

/**
 * @brief calculate distance of two KeyFrame
 * @param k1
 * @param k2
 * @return the distance of two KeyFrame
 */
double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
    Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
    Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
    Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
    double dDist = sqrt(v3Diff * v3Diff);
    return dDist;
}

/**
 * @brief find N nearest KeyFrames from KeyFrame k
 * @param k
 * @param N
 * @return N nearest KeyFrames
 */
vector<KeyFrame*> MapMaker::NClosestKeyFrames(KeyFrame &k, unsigned int N)
{
    vector<pair<double, KeyFrame* > > vKFandScores;
    for (auto &vpKeyFrame : mMap.vpKeyFrames) {
        if(vpKeyFrame == &k)
            continue;
        double dDist = KeyFrameLinearDist(k, *vpKeyFrame);
        vKFandScores.push_back(std::make_pair(dDist, vpKeyFrame));
    }

    if(N > vKFandScores.size())
        N = (unsigned int)vKFandScores.size();
    partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

    vector<KeyFrame*> vResult;
    for(unsigned int i=0; i<N; i++)
        vResult.push_back(vKFandScores[i].second);

    return vResult;
}

/**
 * @brief find the closest KeyFrame from k
 * @param k
 * @return the closest KeyFrame from k
 */
KeyFrame* MapMaker::ClosestKeyFrame(KeyFrame &k)
{
    double dClosestDist = 9999999999.9;
    int nClosest = -1;
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++) {
        if (mMap.vpKeyFrames[i] == &k)
            continue;
        double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
        if (dDist < dClosestDist) {
            dClosestDist = dDist;
            nClosest = i;
        }
    }
    assert(nClosest != -1);
    return mMap.vpKeyFrames[nClosest];
}

bool MapMaker::IsNeedNewKeyFrame(KeyFrame &kCurrent)
{
    KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
    double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
    dDist *= (1.0 / kCurrent.dSceneDepthMean);

    double dDistToCompared = GV2.GetDouble("MapMaker.MaxKFDistWiggleMult", 0.05, SILENT) * mdWiggleScaleDepthNormalized;

    return dDist > dDistToCompared;
}

/**
 * @brief Perform bundle adjustment on all keyframes, all map points
 */
void MapMaker::BundleAdjustAll()
{
    set<KeyFrame*> sAdj;
    set<KeyFrame*> sFixed;
    for (auto &vpKeyFrame : mMap.vpKeyFrames)
        if(vpKeyFrame->bFixed)
            sFixed.insert(vpKeyFrame);
        else
            sAdj.insert(vpKeyFrame);

    set<MapPoint*> sMapPoints;
    for (auto vpPoint : mMap.vpPoints)
        sMapPoints.insert(vpPoint);

    BundleAdjust(sAdj, sFixed, sMapPoints, false);
}

/**
 * @brief Peform a local bundle adjustment which only adjusts recently added key-frames
 */
void MapMaker::BundleAdjustRecent()
{
    if(mMap.vpKeyFrames.size() < 8) { // Ignore this unless map is big enough
        mbBundleConverged_Recent = true;
        return;
    }

    // First, make a list of the keyframes we want adjusted in the adjuster.
    // This will be the last keyframe inserted, and its four nearest neighbors
    set<KeyFrame*> sAdjustSet;
    KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
    sAdjustSet.insert(pkfNewest);
    vector<KeyFrame*> vClosest = NClosestKeyFrames(*pkfNewest, 4);
    for(int i=0; i<4; i++)
        if(!vClosest[i]->bFixed)
            sAdjustSet.insert(vClosest[i]);

    // Now we find the set of features which they contain.
    set<MapPoint*> sMapPoints;
    for (auto iter : sAdjustSet) {
        map<MapPoint*,Measurement> &mKFMeas = iter->mMeasurements;
        for (auto &mKFMea : mKFMeas)
            sMapPoints.insert(mKFMea.first);
    }

    // Finally, add all keyframes which measure above points as fixed keyframes
    set<KeyFrame*> sFixedSet;
    for (auto &vpKeyFrame : mMap.vpKeyFrames) {
        if (sAdjustSet.count(vpKeyFrame))
            continue;
        bool bInclude = false;
        for (auto &mMeasurement : vpKeyFrame->mMeasurements)
            if (sMapPoints.count(mMeasurement.first)) {
                bInclude = true;
                break;
            }
        if (bInclude)
            sFixedSet.insert(vpKeyFrame);
    }

    BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true);
}

/**
 * @brief Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
 * @param sAdjustSet
 * @param sFixedSet
 * @param sMapPoints
 * @param bRecent
 */
void MapMaker::BundleAdjust(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{
    Bundle b(mCamera);   // Our bundle adjuster
    mbBundleRunning = true;
    mbBundleRunningIsRecent = bRecent;

    // The bundle adjuster does different accounting of keyframes and map points;
    // Translation maps are stored:
    map<MapPoint*, int> mPoint_BundleID;
    map<int, MapPoint*> mBundleID_Point;
    map<KeyFrame*, int> mView_BundleID;
    map<int, KeyFrame*> mBundleID_View;

    // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
    for (auto it : sAdjustSet) {
        int nBundleID = b.AddCamera(it->se3CfromW, it->bFixed);
        mView_BundleID[it] = nBundleID;
        mBundleID_View[nBundleID] = it;
    }
    for (auto it : sFixedSet) {
        int nBundleID = b.AddCamera(it->se3CfromW, true);
        mView_BundleID[it] = nBundleID;
        mBundleID_View[nBundleID] = it;
    }

    // Add the points' 3D position
    for (auto sMapPoint : sMapPoints) {
        int nBundleID = b.AddPoint(sMapPoint->v3WorldPos);
        mPoint_BundleID[sMapPoint] = nBundleID;
        mBundleID_Point[nBundleID] = sMapPoint;
    }

    // Add the relevant point-in-keyframe measurements
    for (auto &vpKeyFrame : mMap.vpKeyFrames) {
        if(mView_BundleID.count(vpKeyFrame) == 0)
            continue;
        int nKF_BundleID = mView_BundleID[vpKeyFrame];
        for (auto &mMeasurement : vpKeyFrame->mMeasurements) {
            if(mPoint_BundleID.count(mMeasurement.first) == 0)
                continue;
            int nPoint_BundleID = mPoint_BundleID[mMeasurement.first];
            b.AddMeas(nKF_BundleID, nPoint_BundleID, mMeasurement.second.v2RootPos,
                      Level::LevelScale(mMeasurement.second.nLevel) * Level::LevelScale(mMeasurement.second.nLevel));
        }
    }

    // Run the bundle adjuster. This returns the number of successful iterations
    int nAccepted = b.Compute(&mbBundleAbortRequested);

    if(nAccepted < 0) {
        cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
             << "   The map is probably corrupt: Ditching the map. " << endl;
        mbResetRequested = true;
        return;
    }

    // Bundle adjustment did some updates, apply these to the map
    if(nAccepted > 0) {

        for (auto &itr : mPoint_BundleID)
            itr.first->v3WorldPos = b.GetPoint(itr.second);
        for (auto &itr : mView_BundleID)
            itr.first->se3CfromW = b.GetCamera(itr.second);
        if (bRecent)
            mbBundleConverged_Recent = false;
        mbBundleConverged_Full = false;
    }

    if(b.Converged()) {
        mbBundleConverged_Recent = true;
        if (!bRecent)
            mbBundleConverged_Full = true;
    }

    mbBundleRunning = false;
    mbBundleAbortRequested = false;

    // Handle outlier measurements:
    vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
    for (auto &i : vOutliers_PC_pair) {
        MapPoint *pp = mBundleID_Point[i.first];
        KeyFrame *pk = mBundleID_View[i.second];
        Measurement &m = pk->mMeasurements[pp];
        if (pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT)   // Is the original source kf considered an outlier? That's bad.
            pp->bBad = true;
        else {
            // Do we retry it? Depends where it came from!!
            if (m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
                mvFailureQueue.push_back(pair<KeyFrame *, MapPoint *>(pk, pp));
            else
                pp->pMMData->sNeverRetryKFs.insert(pk);
            pk->mMeasurements.erase(pp);
            pp->pMMData->sMeasurementKFs.erase(pk);
        }
    }
}

/**
 * @brief Mapmaker's try-to-find-a-point-in-a-keyframe code
 * @param k
 * @param p
 * @return
 * @details This is used to update data association if a bad measurement was detected,
 *          or if a point was never searched for in a keyframe in the first place.
 */
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p)
{
    // abort if either a measurement is already in the map, or we've
    // decided that this point-kf combo is beyond redemption
    if(p.pMMData->sMeasurementKFs.count(&k) || p.pMMData->sNeverRetryKFs.count(&k))
        return false;

    Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
    if(v3Cam[2] < 0.001)
    {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }
    Vector<2> v2ImPlane = project(v3Cam);
    if(v2ImPlane* v2ImPlane > mCamera.LargestRadiusInImage() * mCamera.LargestRadiusInImage())
    {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    Vector<2> v2Image = mCamera.Project(v2ImPlane);
    if(mCamera.Invalid())
    {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    ImageRef irImageSize = k.aLevels[0].im.size();
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
    {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    static PatchFinder Finder;
    Matrix<2> m2CamDerivs = mCamera.GetProjectionDerivs();
    Finder.CalcSearchLevelAndWarpMatrix(p, k.se3CfromW, m2CamDerivs);
    Finder.MakeTemplateCoarseCont(p);

    if(Finder.TemplateBad())
    {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    bool bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!
    if(!bFound)
    {
        p.pMMData->sNeverRetryKFs.insert(&k);
        return false;
    }

    // If we found something, generate a measurement struct and put it in the map
    Measurement m;
    m.nLevel = Finder.GetLevel();
    m.Source = Measurement::SRC_REFIND;

    if(Finder.GetLevel() > 0)
    {
        Finder.MakeSubPixTemplate();
        Finder.IterateSubPixToConvergence(k,8);
        m.v2RootPos = Finder.GetSubPixPos();
        m.bSubPix = true;
    }
    else
    {
        m.v2RootPos = Finder.GetCoarsePosAsVector();
        m.bSubPix = false;
    }

    if(k.mMeasurements.count(&p))
    {
        assert(false); // This should never happen, we checked for this at the start.
    }
    k.mMeasurements[&p] = m;
    p.pMMData->sMeasurementKFs.insert(&k);
    return true;
}

/**
 * @brief A general data-association update for a single keyframe,
 *        Do this on a new key-frame when it's passed in by the tracker
 * @param k
 * @return
 */
int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k)
{
    vector<MapPoint*> vToFind;
    for (auto vpPoint : mMap.vpPoints)
        vToFind.push_back(vpPoint);

    int nFoundNow = 0;
    for (auto &i : vToFind)
        if(ReFind_Common(k,*i))
            nFoundNow++;

    return nFoundNow;
}

/**
 * @brief When new map points are generated, they're only created from a stereo pair
 *        this tries to make additional measurements in other KFs which they might be in.
 */
void MapMaker::ReFindNewlyMade()
{
    if(mqNewQueue.empty())
        return;

    int nFound = 0;
    int nBad = 0;
    while(!mqNewQueue.empty() && mvpKeyFrameQueue.empty()) {
        MapPoint *pNew = mqNewQueue.front();
        mqNewQueue.pop();
        if (pNew->bBad) {
            nBad++;
            continue;
        }
        for (auto &vpKeyFrame : mMap.vpKeyFrames) {
            if (ReFind_Common(*vpKeyFrame, *pNew))
                nFound++;
        }
    }
}

/**
 * @brief Dud measurements get a second chance
 */
void MapMaker::ReFindFromFailureQueue()
{
    if(mvFailureQueue.empty())
        return;
    std::sort(mvFailureQueue.begin(), mvFailureQueue.end());
    int nFound=0;
    for (auto &it : mvFailureQueue)
        if(ReFind_Common(*it.first, *it.second))
            nFound++;

    mvFailureQueue.erase(mvFailureQueue.begin(), mvFailureQueue.end());
}

/**
 * @brief Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
 * @return
 * @details
 * Steps : \n
 * 1) Get inliner points vv3Inliers from mass of v3WorldPos of mMap.vpPoints by RANSAC; \n
 * 2) Get the center of mass v3MeanOfInliers from vv3Inliers from above,
 *    and make it as the origin of new coordinate system; \n
 * 3) Calculate Covariance Matrix by v3MeanOfInliers and vv3Inliers; \n
 * 4) Find the principal component with the minimal variance: this is the plane normal v3Normal; \n
 * 5) Calculate Rotation Matrix by Gram-Schmidt Orthonormalization with v3Normal; \n
 * 6) Calculate Translate Vector by:
 *    \f[
 *       P_2 = R(P_1-C) = RP_1 - RC
 *    \f]
 *    where \f$ C = v3MeanOfInliers \f$ ; so, \f$ T = -R \cdot C \f$
 */
SE3<> MapMaker::CalcPlaneAligner()
{
    auto nPoints = static_cast<unsigned int>(mMap.vpPoints.size());
    if(nPoints < 10) {
        cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
        return SE3<>();
    }

    Vector<3> v3BestMean;
    Vector<3> v3BestNormal;
    double dBestDistSquared = 9999999999999999.9;
    int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
    for(int i=0; i<nRansacs; i++) {
        int nA = rand() % nPoints;
        int nB = nA;
        int nC = nA;
        while (nB == nA)
            nB = rand() % nPoints;
        while (nC == nA || nC == nB)
            nC = rand() % nPoints;

        Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos +
                                         mMap.vpPoints[nB]->v3WorldPos +
                                         mMap.vpPoints[nC]->v3WorldPos);

        Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos - mMap.vpPoints[nA]->v3WorldPos;
        Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos - mMap.vpPoints[nA]->v3WorldPos;
        Vector<3> v3Normal = v3CA ^ v3BA;
        if (v3Normal * v3Normal == 0)
            continue;
        normalize(v3Normal);

        double dSumError = 0.0;
        for (unsigned int i = 0; i < nPoints; i++) {
            Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
            double dDistSq = v3Diff * v3Diff;
            if (dDistSq == 0.0)
                continue;
            double dNormDist = fabs(v3Diff * v3Normal);

            if (dNormDist > 0.05)
                dNormDist = 0.05;
            dSumError += dNormDist;
        }
        if (dSumError < dBestDistSquared) {
            dBestDistSquared = dSumError;
            v3BestMean = v3Mean;
            v3BestNormal = v3Normal;
        }
    }

    // Done the ransacs, now collect the supposed inlier set
    vector<Vector<3> > vv3Inliers;
    for(unsigned int i=0; i<nPoints; i++) {
        Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
        double dDistSq = v3Diff * v3Diff;
        if (dDistSq == 0.0)
            continue;
        double dNormDist = fabs(v3Diff * v3BestNormal);
        if (dNormDist < 0.05)
            vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
    }

    // With these inliers, calculate mean and cov
    Vector<3> v3MeanOfInliers = Zeros;
    for (const auto &vv3Inlier : vv3Inliers)
        v3MeanOfInliers += vv3Inlier;
    v3MeanOfInliers *= (1.0 / vv3Inliers.size());

    Matrix<3> m3Cov = Zeros;
    for (const auto &vv3Inlier : vv3Inliers) {
        Vector<3> v3Diff = vv3Inlier - v3MeanOfInliers;
        m3Cov += v3Diff.as_col() * v3Diff.as_row();
    }

    // Find the principal component with the minimal variance: this is the plane normal
    SymEigen<3> sym(m3Cov);
    Vector<3> v3Normal = sym.get_evectors()[0];

    // Use the version of the normal which points towards the cam center
    if(v3Normal[2] > 0)
        v3Normal *= -1.0;

    Matrix<3> m3Rot = Identity;
    m3Rot[2] = v3Normal;
    m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
    normalize(m3Rot[0]);
    m3Rot[1] = m3Rot[2] ^ m3Rot[0];

    SE3<> se3Aligner;
    se3Aligner.get_rotation() = m3Rot;
    Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
    se3Aligner.get_translation() = -v3RMean;

    return se3Aligner;
}

/**
 * @brief Calculates the depth(z-) distribution of map points visible in a keyframe
 *        This function is only used for the first two keyframes, all others get this filled in by the tracker
 * @param pKF
 */
void MapMaker::RefreshSceneDepth(KeyFrame *pKF)
{
    double dSumDepth = 0.0;
    double dSumDepthSquared = 0.0;
    int nMeas = 0;
    for(auto it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
        MapPoint &point = *it->first;
        Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
        dSumDepth += v3PosK[2];
        dSumDepthSquared += v3PosK[2] * v3PosK[2];
        nMeas++;
    }

    assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
    pKF->dSceneDepthMean = dSumDepth / nMeas;
    pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
    Command c;
    c.sCommand = sCommand;
    c.sParams = sParams;
    ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
    if(sCommand=="SaveMap")
    {
        cout << "  MapMaker: Saving the map.... " << endl;
        ofstream ofs("map.dump");
        for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
        {
            ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
            ofs << mMap.vpPoints[i]->nSourceLevel << endl;
        }
        ofs.close();

        for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
        {
            ostringstream ost1;
            ost1 << "keyframes/" << i << ".jpg";
            //	  img_save(mMap.vpKeyFrames[i]->aLevels[0].im, ost1.str());

            ostringstream ost2;
            ost2 << "keyframes/" << i << ".info";
            ofstream ofs2;
            ofs2.open(ost2.str().c_str());
            ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
            ofs2.close();
        }
        cout << "  ... done saving map." << endl;
        return;
    }

    cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
    exit(1);
}












