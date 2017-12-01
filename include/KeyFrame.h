// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/utility.h>
#include <vector>
#include <set>
#include <map>

using namespace TooN;

class MapPoint;
class SmallBlurryImage;

#define LEVELS 4

// Candidate: a feature in an image which could be made into a map point
struct Candidate
{
    CVD::ImageRef irLevelPos;
    Vector<2> v2RootPos;
    double dSTScore;
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
struct Measurement
{
    int nLevel;   // Which image level?
    bool bSubPix; // Has this measurement been refined to sub-pixel level?
    Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
    enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

// Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
// This contains image data and corner points.
struct Level
{
    inline Level() {
        bImplaneCornersCached = false;
    }

    CVD::Image<CVD::byte> im;                // The pyramid level pixels
    std::vector<CVD::ImageRef> vCorners;     // All FAST corners on this level
    std::vector<int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
    std::vector<CVD::ImageRef> vMaxCorners;  // The maximal FAST corners
    // The keyframe struct is quite happy with default operator=, but Level needs its own to override CVD's reference-counting behaviour.
    Level& operator=(const Level &rhs) {
        // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
        im.resize(rhs.im.size());
        copy(rhs.im, im);

        vCorners = rhs.vCorners;
        vMaxCorners = rhs.vMaxCorners;
        vCornerRowLUT = rhs.vCornerRowLUT;
        return *this;
    }

    std::vector<Candidate> vCandidates;   // Potential locations of new map points

    bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
    std::vector<Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates

    static Vector<3> mvLevelColors[];

    // What is the scale of a level?
    inline static int LevelScale(int nLevel) {
        return 1 << nLevel;
    }

    // 1-D transform to level zero:
    inline static double LevelZeroPos(double dLevelPos, int nLevel)
    {
        return (dLevelPos + 0.5) * LevelScale(nLevel) - 0.5;
    }

    // 2-D transforms to level zero:
    inline static Vector<2> LevelZeroPos(Vector<2> v2LevelPos, int nLevel)
    {
        Vector<2> v2Ans;
        v2Ans[0] = LevelZeroPos(v2LevelPos[0], nLevel);
        v2Ans[1] = LevelZeroPos(v2LevelPos[1], nLevel);
        return v2Ans;
    }
    inline static Vector<2> LevelZeroPos(CVD::ImageRef irLevelPos, int nLevel)
    {
        Vector<2> v2Ans;
        v2Ans[0] = LevelZeroPos(irLevelPos.x, nLevel);
        v2Ans[1] = LevelZeroPos(irLevelPos.y, nLevel);
        return v2Ans;
    }

    // 1-D transform from level zero to level N:
    inline static double LevelNPos(double dRootPos, int nLevel)
    {
        return (dRootPos + 0.5) / LevelScale(nLevel) - 0.5;
    }

    // 2-D transform from level zero to level N:
    inline static Vector<2> LevelNPos(Vector<2> v2RootPos, int nLevel)
    {
        Vector<2> v2Ans;
        v2Ans[0] = LevelNPos(v2RootPos[0], nLevel);
        v2Ans[1] = LevelNPos(v2RootPos[1], nLevel);
        return v2Ans;
    }
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these 
// are then simply discarded, but sometimes they're then just added to the map.
struct KeyFrame
{
    inline KeyFrame()
    {
        pSBI = NULL;
    }
    SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation
    bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)
    Level aLevels[LEVELS];  // Images, corners, etc lives in this array of pyramid levels
    std::map<MapPoint*, Measurement> mMeasurements;           // All the measurements associated with the keyframe

    void MakeKeyFrame_Lite(CVD::BasicImage<CVD::byte> &im);   // This takes an image and calculates pyramid levels etc to fill the
    // keyframe data structures with everything that's needed by the tracker..
    void MakeKeyFrame_Rest();                                 // ... while this calculates the rest of the data which the mapmaker needs.

    double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.
    double dSceneDepthSigma;

    SmallBlurryImage *pSBI; // The relocaliser uses this
};

#endif

