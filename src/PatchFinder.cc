// Copyright 2008 Isis Innovation Limited
#include "PatchFinder.h"

#include <cvd/vision.h>
#include <cvd/image_interpolate.h>
#include <TooN/Cholesky.h>

#include "Tools.h"
#include "ImageProcess.h"

using namespace CVD;
using namespace std;

PatchFinder::PatchFinder(int nPatchSize) : mimTemplate(ImageRef(nPatchSize,nPatchSize))
{
    mnPatchSize = nPatchSize;
    mirCenter = ImageRef(nPatchSize/2, nPatchSize/2);
    int nMaxSSDPerPixel = 500; // Pretty arbitrary... could make a GVar out of this.
    mnMaxSSD = mnPatchSize * mnPatchSize * nMaxSSDPerPixel;
    // Populate the speed-up caches with bogus values:
    mm2LastWarpMatrix = 9999.9 * TooN::Identity;
    mpLastTemplateMapPoint = nullptr;
}

/**
 * @brief Find the warping matrix and search level
 * @param p
 * @param se3CFromW
 * @param m2CamDerivs
 * @return mnSearchLevel
 * @details
 * (1) \f$ \{u_s,v_s\} \f$ 对应源帧金字塔层水平和竖直方向的像素位移，\f$ \{u_c,v_c\} \f$ 对应当前帧金字塔0层水平和竖直方向的像素位移，
 *     \f$ u_c,v_c \f$ 分别为 \f$ u_s,v_s \f$ 的函数，则 \f$ u_c,v_c \f$ 的微分（矩阵形式）为：
 *     \f[
 *        \begin{bmatrix} du_c \\ dv_c \end{bmatrix} =
 *        \begin{bmatrix}
 *              \frac{\partial{u_c}}{\partial{u_s}} & \frac{\partial{u_c}}{\partial{v_s}} \\
 *              \frac{\partial{v_c}}{\partial{u_s}} & \frac{\partial{v_c}}{\partial{v_s}}
 *        \end{bmatrix}
 *        \begin{bmatrix} du_s \\ dv_s \end{bmatrix}
 *     \f]
 * (2) Affine Warp Matrix 为：
 *     \f[
 *        A =
 *        \begin{bmatrix}
 *              \frac{\partial{u_c}}{\partial{u_s}} & \frac{\partial{u_c}}{\partial{v_s}} \\
 *              \frac{\partial{v_c}}{\partial{u_s}} & \frac{\partial{v_c}}{\partial{v_s}}
 *        \end{bmatrix}
 *     \f]
 * (3) 行列式\f$ |A^{-1}| \f$ 的几何意义： 向量 \f$ du_s , dv_s \f$ 与 向量 \f$ du_c , dv_c \f$ 分别张成的平行四边形的面积之比
 */
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint &p, SE3<> se3CFromW, Matrix<2> &m2CamDerivs)
{
    // Calc point pos in new view camera frame
    // Slightly dumb that we re-calculate this here when the tracker's already done this!
    Vector<3> v3Cam = se3CFromW * p.v3WorldPos;
    double dOneOverCameraZ = 1.0 / v3Cam[2];

    // Project the source keyframe's one-pixel-right and one-pixel-down vectors into the current view
    Vector<3> v3MotionRight = se3CFromW.get_rotation() * p.v3PixelRight_W;
    Vector<3> v3MotionDown  = se3CFromW.get_rotation() * p.v3PixelDown_W;

    // Calculate in-image derivatives of source image pixel motions:
    mm2WarpInverse.T()[0] = m2CamDerivs * (v3MotionRight.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionRight[2] * dOneOverCameraZ) * dOneOverCameraZ;
    mm2WarpInverse.T()[1] = m2CamDerivs * (v3MotionDown.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionDown[2] * dOneOverCameraZ) * dOneOverCameraZ;

    double dDet = mm2WarpInverse[0][0] * mm2WarpInverse[1][1] - mm2WarpInverse[0][1] * mm2WarpInverse[1][0];
    mnSearchLevel = 0;
    // This warp matrix is likely not appropriate for finding at level zero, which is
    // the level at which it has been calculated. Vary the search level until the
    // at that level would be appropriate (does not actually modify the matrix.)
    while(dDet > 3 && mnSearchLevel < LEVELS-1) {
        mnSearchLevel++;
        dDet *= 0.25;
    }

    // Some warps are inappropriate, e.g. too near the camera, too far, or reflected, or zero area.. reject these!
    if(dDet > 3 || dDet < 0.25) {
        mbTemplateBad = true;
        return -1;
    }
    else
        return mnSearchLevel;
}

/**
 * @brief generates the warped search template
 * @param p a MapPoint
 * @details
 * @verbatim
 * Optimisation:
 * Don't re-gen the coarse template if it's going to be substantially the same as was made last time.
 * This saves time when the camera is not moving. For this, check that:
 *   (a) this patchfinder is still working on the same map point;
 *   (b) the warping matrix has not changed much.
 * @endverbatim
 */
void PatchFinder::MakeTemplateCoarseCont(MapPoint &p)
{
    // m2 represents the number of pixels in the source image for one pixel of template image
    Matrix<2> m2 = cg::Tools::M2Inverse(mm2WarpInverse) * Level::LevelScale(mnSearchLevel);

    bool bNeedToRefreshTemplate = &p!=mpLastTemplateMapPoint; //Still the same map point?

    for(int i=0; !bNeedToRefreshTemplate && i<2; i++) {
        Vector<2> v2Diff = m2.T()[i] - mm2LastWarpMatrix.T()[i];
        const double dRefreshLimit = 0.07;  // Sort of works out as half a pixel displacement in src img
        if (v2Diff * v2Diff > dRefreshLimit * dRefreshLimit)
            bNeedToRefreshTemplate = true;
    }

    // Need to regen template? Then go ahead.
    if(bNeedToRefreshTemplate) {
        // Use CVD::transform to warp the patch according the the warping matrix m2
        // This returns the number of pixels outside the source image hit, which should be zero.
        int nOutside = CVD::transform(p.pPatchSourceKF->aLevels[p.nSourceLevel].im,
                                  mimTemplate,
                                  m2,
                                  vec(p.irCenter),
                                  vec(mirCenter));
        mbTemplateBad = (bool)nOutside;
        MakeTemplateSums();
        // Store the parameters which allow us to determine if we need to re-calculate the patch next time round.
        mpLastTemplateMapPoint = &p;
        mm2LastWarpMatrix = m2;
    }
}

/**
 * @brief makes a template without warping
 * @param k KeyFrame
 * @param nLevel
 * @param irLevelPos
 * @details Used for epipolar search, where we don't really know what the warping matrix should be.
 *          (Although to be fair, I should do rotation for epipolar, which we could approximate without knowing patch depth!)
 */
void PatchFinder::MakeTemplateCoarseNoWarp(KeyFrame &k, int nLevel, ImageRef irLevelPos)
{
    mnSearchLevel = nLevel;
    Image<byte> &im = k.aLevels[nLevel].im;
    if(!im.in_image_with_border(irLevelPos, mnPatchSize / 2 + 1)) {
        mbTemplateBad = true;
        return;
    }
    mbTemplateBad = false;
    copy(im, mimTemplate, mimTemplate.size(), irLevelPos - mirCenter);
    MakeTemplateSums();
}

/**
 * @brief Looks at the appropriate level of the target keyframe to try and find the template.
 * @param irPos
 * @param kf
 * @param nRange
 * @return Returns true on patch found
 * @details Looks only at FAST corner points which are within radius nRange of the center.\n
 *          It's a bit optimised to use a corner row look-up-table, since otherwise the routine
 *          would spend a long time trawling throught the whole list of FAST corners!
 */
bool PatchFinder::FindPatchCoarse(ImageRef irPos, KeyFrame &kf, unsigned int nRange)
{
    mbFound = false;

    // Convert from L0 coords to search level quantities
    int nLevelScale = Level::LevelScale(mnSearchLevel);
    mirPredictedPos = irPos;
    irPos = irPos / nLevelScale;
    nRange = (nRange + nLevelScale - 1) / nLevelScale;

    // Bounding box of search circle
    int nTop = irPos.y - nRange;
    int nBottomPlusOne = irPos.y + nRange + 1;
    int nLeft = irPos.x - nRange;
    int nRight = irPos.x + nRange;

    // Ref variable for the search level
    Level &L = kf.aLevels[mnSearchLevel];

    // Some bounds checks on the bounding box..
    if(nTop < 0)
        nTop = 0;
    if(nTop >= L.im.size().y)
        return false;
    if(nBottomPlusOne <= 0)
        return false;

    // The next section finds all the FAST corners in the target level which are near enough the search center.
    ImageRef irBest;             // Best match so far
    int nBestSSD = mnMaxSSD + 1; // Best score so far is beyond the max allowed
    auto i = L.vCorners.begin() + L.vCornerRowLUT[nTop];
    auto i_end = nBottomPlusOne >= L.im.size().y ? L.vCorners.end() : L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];
    for(; i<i_end; i++) {
        if (i->x < nLeft || i->x > nRight)
            continue;
        if ((irPos - *i).mag_squared() > nRange * nRange)
            continue;
        int nSSD = ZMSSDAtPoint(L.im, *i);
        if (nSSD < nBestSSD) {
            irBest = *i;
            nBestSSD = nSSD;
        }
    }
    if(nBestSSD < mnMaxSSD) {
        mv2CoarsePos = Level::LevelZeroPos(irBest, mnSearchLevel);
        mbFound = true;
    }
    else
        mbFound = false;

    return mbFound;
}

/**
 * @brief Makes an inverse composition template out of the coarse template.
 * @details Includes calculating image of derivatives (gradients.)
 *          The inverse composition used here operates on three variables: x offet, y offset, and difference in patch means;
 *          hence things like mm3HInv are dim 3, but the trivial mean jacobian (always unity, for each pixel) is not stored.
 */
void PatchFinder::MakeSubPixTemplate()
{
    mimJacs.resize(mimTemplate.size() - ImageRef(2,2));
    Matrix<3> m3H = Zeros; // This stores jTj.
    ImageRef ir;
    for(ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
        for(ir.y = 1; ir.y < mnPatchSize - 1; ir.y++) {
            Vector<2> v2Grad;
            v2Grad[0] = 0.5 * (mimTemplate[ir + ImageRef(1, 0)] - mimTemplate[ir - ImageRef(1, 0)]);
            v2Grad[1] = 0.5 * (mimTemplate[ir + ImageRef(0, 1)] - mimTemplate[ir - ImageRef(0, 1)]);
            mimJacs[ir - ImageRef(1, 1)].first = static_cast<float>(v2Grad[0]);
            mimJacs[ir - ImageRef(1, 1)].second = static_cast<float>(v2Grad[1]);
            Vector<3> v3Grad = unproject(v2Grad); // This adds the mean-difference jacobian..
            m3H += v3Grad.as_col() * v3Grad.as_row(); // Populate JTJ.
        }

    Cholesky<3> chol(m3H);
    mm3HInv = chol.get_inverse();// Invert JTJ..

    mv2SubPixPos = mv2CoarsePos; // Start the sub-pixel search at the result of the coarse search..
    mdMeanDiff = 0.0;
}

/**
 * @details Iterate inverse composition until convergence.
 *          Since it should never have to travel more than a pixel's distance, set a max number of iterations;
 *          if this is exceeded, consider the IC to have failed.
 * @param kf
 * @param nMaxIts
 * @return
 */
bool PatchFinder::IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts)
{
    const double dConvLimit = 0.03;
    for(int nIts = 0; nIts < nMaxIts; nIts++) {
        double dUpdateSquared = IterateSubPix(kf);
        if (dUpdateSquared < 0) // went off edge of image
            return false;
        if (dUpdateSquared < dConvLimit * dConvLimit)
            return true;
    }
    return false;
}

/**
 * @brief Single iteration of inverse composition
 * @param kf
 * @return
 * @details This compares integral image positions in the template image to floating point positions in the target keyframe.
 *          Each template pixel will be compared to an interpolated target pixel. \n
 *          The target value is made using [bilinear interpolation](https://en.wikipedia.org/wiki/Bilinear_interpolation)
 *          as the weighted sum of four target image pixels. \n
 *          Interpolation is bilinear, and performed manually (rather than using CVD::image_interpolate)
 *          since this is a special case where the mixing fractions for each pixel are identical.
 *
 */
double PatchFinder::IterateSubPix(KeyFrame &kf)
{
    Vector<2> v2Center = Level::LevelNPos(mv2SubPixPos, mnSearchLevel);
    BasicImage<byte> &im = kf.aLevels[mnSearchLevel].im;
    if(!im.in_image_with_border(ir_rounded(v2Center), mnPatchSize / 2 + 1))
        return -1.0;

    Vector<2> v2Base = v2Center - vec(mirCenter); //Position of top-left corner of patch in search level

    // Calculate mixing fractions:
    double dX = v2Base[0]-floor(v2Base[0]); // Distances from pixel center of TL pixel
    double dY = v2Base[1]-floor(v2Base[1]);
    auto fMixTL = static_cast<float>((1.0 - dX) * (1.0 - dY));
    auto fMixTR = static_cast<float>((dX) * (1.0 - dY));
    auto fMixBL = static_cast<float>((1.0 - dX) * (dY));
    auto fMixBR = static_cast<float>((dX) * (dY));

    ImageRef ir;
    Vector<3> v3Accum = Zeros; // I.C. JT*d accumulator
    unsigned long nRowOffset = &kf.aLevels[mnSearchLevel].im[ImageRef(0,1)] - &kf.aLevels[mnSearchLevel].im[ImageRef(0,0)];
    for(ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
    {
        byte* pTopLeftPixel = &im[::ir(v2Base) + ImageRef(1,ir.y)]; // n.b. the x=1 offset, as with y
        for(ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
        {
            float fPixel =   // Calc target interpolated pixel
                    fMixTL * pTopLeftPixel[0]          + fMixTR * pTopLeftPixel[1] +
                    fMixBL * pTopLeftPixel[nRowOffset] + fMixBR * pTopLeftPixel[nRowOffset + 1];
            pTopLeftPixel++;
            double dDiff = fPixel - mimTemplate[ir] + mdMeanDiff;
            v3Accum[0] += dDiff * mimJacs[ir - ImageRef(1,1)].first;
            v3Accum[1] += dDiff * mimJacs[ir - ImageRef(1,1)].second;
            v3Accum[2] += dDiff;  // Update JT*d
        }
    }

    // All done looping over image - find JTJ^-1 * JTd:
    Vector<3> v3Update = mm3HInv * v3Accum;
    mv2SubPixPos -= v3Update.slice<0,2>() * Level::LevelScale(mnSearchLevel);
    mdMeanDiff -= v3Update[2];

    double dPixelUpdateSquared = v3Update.slice<0,2>() * v3Update.slice<0,2>();
    return dPixelUpdateSquared;
}

/**
 * @brief Calculate the Zero-mean SSD of the coarse patch and a target imate at a specific point
 * @param im
 * @param ir
 * @return value
 */
int PatchFinder::ZMSSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir)
{
    return ImageProcess::ZMSSDAtPoint(im, ir, mimTemplate, mnTemplateSum, mnTemplateSumSq, mnMaxSSD);
}
