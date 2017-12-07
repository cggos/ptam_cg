// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"

#include <cvd/vision.h>
#include <cvd/fast_corner.h>

#include "ImageProcess.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

/**
 * @brief Perpares a Keyframe from an image.
 *        Generates pyramid levels, does FAST detection, etc.
 * @param im gray scale image
 */
void KeyFrame::MakeKeyFrame_Lite(BasicImage<byte> &im)
{
    aLevels[0].im.resize(im.size());
    copy(im, aLevels[0].im);

    for(int i=0; i<LEVELS; i++) {
        Level &lev = aLevels[i];
        if (i != 0) {
            lev.im.resize(aLevels[i - 1].im.size() / 2);
            halfSample(aLevels[i - 1].im, lev.im);
        }
        // .. and detect and store FAST corner points.
        // I use a different threshold on each level; this is a bit of a hack
        // whose aim is to balance the different levels' relative feature densities.
        lev.vCorners.clear();
        lev.vCandidates.clear();
        lev.vMaxCorners.clear();
        if (i == 0)
            fast_corner_detect_10(lev.im, lev.vCorners, 10);
        if (i == 1)
            fast_corner_detect_10(lev.im, lev.vCorners, 15);
        if (i == 2)
            fast_corner_detect_10(lev.im, lev.vCorners, 15);
        if (i == 3)
            fast_corner_detect_10(lev.im, lev.vCorners, 10);

        // Generate row look-up-table for the FAST corner points: this speeds up
        // finding close-by corner points later on.
        unsigned int v = 0;
        lev.vCornerRowLUT.clear();
        for (int y = 0; y < lev.im.size().y; y++) {
            while (v < lev.vCorners.size() && y > lev.vCorners[v].y)
                v++;
            lev.vCornerRowLUT.push_back(v);
        }
    }
}

/**
 * @brief Fills the rest of the keyframe structure needed by the mapmaker:
 *        FAST nonmax suppression, generation of the list of candidates for further map points,
 *        creation of the relocaliser's SmallBlurryImage.
 */
void KeyFrame::MakeKeyFrame_Rest()
{
    static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);

    for (auto &lev : aLevels) {
        fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
        for (auto i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++) {
            if (!lev.im.in_image_with_border(*i, 10))
                continue;
            double dSTScore = ImageProcess::ShiTomasiScoreAtPoint(lev.im, 3, *i);
            if (dSTScore > *gvdCandidateMinSTScore) {
                Candidate c;
                c.irLevelPos = *i;
                c.dSTScore = dSTScore;
                lev.vCandidates.push_back(c);
            }
        }
    }

    pSBI = new SmallBlurryImage(*this);
    pSBI->MakeJacs(pSBI->mimTemplate, pSBI->mimImageJacs);
}

Vector<3> Level::mvLevelColors[] = {
        makeVector( 1.0, 0.0, 0.0),
        makeVector( 1.0, 1.0, 0.0),
        makeVector( 0.0, 1.0, 0.0),
        makeVector( 0.0, 0.0, 0.7),
        makeVector( 0.0, 0.0, 0.7)
};





