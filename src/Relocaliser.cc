// Copyright 2008 Isis Innovation Limited
#include "Relocaliser.h"

#include <gvars3/instances.h>

#include "ImageProcess.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

bool Relocaliser::AttemptRecovery(KeyFrame &kCurrent)
{
    // Ensure the incoming frame has a SmallBlurryImage attached
    if(!kCurrent.pSBI)
        kCurrent.pSBI = new SmallBlurryImage(kCurrent);
    else
        kCurrent.pSBI->MakeFromKF(kCurrent);

    // Compare current KF to all KFs stored in map by Zero-mean SSD and Find the best ZMSSD match
    mdBestScore = 99999999999999.9;
    mnBest = -1;
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
        double dSSD = kCurrent.pSBI->SSDofImgs(kCurrent.pSBI->mimTemplate, mMap.vpKeyFrames[i]->pSBI->mimTemplate);
        if(dSSD < mdBestScore)
        {
            mdBestScore = dSSD;
            mnBest = i;
        }
    }

    // And estimate a camera rotation from a 3DOF image alignment
    pair<SE2<>, double> result_pair = kCurrent.pSBI->IteratePosRelToTarget(*mMap.vpKeyFrames[mnBest]->pSBI, 6);
    mse2 = result_pair.first;
    double dScore =result_pair.second;
    SE3<> se3KeyFramePos = mMap.vpKeyFrames[mnBest]->se3CfromW;
    mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos;

    return dScore < GV2.GetDouble("Reloc2.MaxScore", 9e6, SILENT);
}

