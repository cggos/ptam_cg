// -*- c++ -*- 
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#ifndef __RELOCALISER_H
#define __RELOCALISER_H

#include <TooN/TooN.h>
#include <TooN/se2.h>
#include <TooN/se3.h>

#include "ATANCamera.h"
#include "Map.h"

using namespace TooN;

class Relocaliser
{
public:
    Relocaliser(Map &map, ATANCamera &camera):mMap(map),mCamera(camera){}
    bool AttemptRecovery(KeyFrame &k);
    SE3<> BestPose(){ return mse3Best; }

protected:
    Map &mMap;
    ATANCamera mCamera;
    int mnBest;
    double mdBestScore;
    SE3<> mse3Best;
};
#endif









