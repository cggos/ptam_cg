#include "ImageProcess.h"

#include <math.h>

using namespace std;

CVD::Image<CVD::byte> ImageProcess::GetImageROI(BasicImage<byte> &im, ImageRef irPos, CVD::ImageRef irSize)
{
    CVD::Image<CVD::byte> imSample;
    assert(im.in_image_with_border(irPos, irSize.x/2) || im.in_image_with_border(irPos, irSize.y/2));
    imSample.resize(irSize);
    copy(im, imSample, irSize, irPos-irSize/2);
    return imSample;
}

double ImageProcess::ShiTomasiScoreAtPoint(BasicImage<byte> &image, int nHalfBoxSize, ImageRef irCenter)
{
    double dXX = 0;
    double dYY = 0;
    double dXY = 0;

    ImageRef irStart = irCenter - ImageRef(nHalfBoxSize, nHalfBoxSize);
    ImageRef irEnd = irCenter + ImageRef(nHalfBoxSize, nHalfBoxSize);

    ImageRef ir;
    for(ir.y = irStart.y; ir.y<=irEnd.y; ir.y++)
        for(ir.x = irStart.x; ir.x<=irEnd.x; ir.x++)
        {
            double dx = image[ir + ImageRef(1,0)] - image[ir - ImageRef(1,0)];
            double dy = image[ir + ImageRef(0,1)] - image[ir - ImageRef(0,1)];
            dXX += dx*dx;
            dYY += dy*dy;
            dXY += dx*dy;
        }

    int nPixels = (irEnd - irStart + ImageRef(1,1)).area();
    dXX = dXX / (2.0 * nPixels);
    dYY = dYY / (2.0 * nPixels);
    dXY = dXY / (2.0 * nPixels);

    // Find and return smaller eigenvalue:
    return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}

// Scoring function
inline int ImageProcess::SSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir, CVD::Image<CVD::byte> &imTemplate, int nMaxSSD)
{
    if(!im.in_image_with_border(ir, imTemplate.size().x/2) || !im.in_image_with_border(ir, imTemplate.size().y/2))
        return nMaxSSD + 1;
    ImageRef irImgBase = ir - imTemplate.size()/2;
    int nRows = imTemplate.size().y;
    int nCols = imTemplate.size().x;
    byte *pImg;
    byte *pTem;
    int nDiff;
    int nSumSqDiff = 0;
    for(int nRow = 0; nRow < nRows; nRow++)
    {
        pImg = &im[irImgBase+ImageRef(0,nRow)];
        pTem = &imTemplate[ImageRef(0,nRow)];
        for(int nCol = 0; nCol < nCols; nCol++)
        {
            nDiff = pImg[nCol] - pTem[nCol];
            nSumSqDiff += nDiff * nDiff;
        };
    };
    return nSumSqDiff;
}

// Define the patch from an input image
void MiniPatch::SampleFromImage(ImageRef irPos, BasicImage<byte> &im)
{
    mimOrigPatch = GetImageROI(im, irPos, mirPatchSize);
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
bool MiniPatch::FindPatch(CVD::ImageRef &irPos,
                          CVD::BasicImage<CVD::byte> &im,
                          int nRange,
                          vector<ImageRef> &vCorners,
                          int nMaxSSD,
                          std::vector<int> *pvRowLUT)
{
    ImageRef irCenter = irPos;
    ImageRef irBest;
    int nBestSSD = nMaxSSD + 1;
    ImageRef irBBoxTL = irPos - ImageRef(nRange, nRange);
    ImageRef irBBoxBR = irPos + ImageRef(nRange, nRange);
    vector<ImageRef>::iterator i;
    if(!pvRowLUT)
    {
        for(i = vCorners.begin(); i!=vCorners.end(); i++)
            if(i->y >= irBBoxTL.y) break;
    }
    else
    {
        int nTopRow = irBBoxTL.y;
        if(nTopRow < 0)
            nTopRow = 0;
        if(nTopRow >= (int) pvRowLUT->size())
            nTopRow = (int) pvRowLUT->size() - 1;
        i = vCorners.begin() + (*pvRowLUT)[nTopRow];
    }

    for(; i!=vCorners.end(); i++)
    {
        if(i->x < irBBoxTL.x  || i->x > irBBoxBR.x)
            continue;
        if(i->y > irBBoxBR.y)
            break;
        int nSSD = SSDAtPoint(im, *i, mimOrigPatch, nMaxSSD);

        if(nSSD < nBestSSD)
        {
            irBest = *i;
            nBestSSD = nSSD;
        }
    }
    if(nBestSSD < nMaxSSD)
    {
        irPos = irBest;
        return true;
    }
    else
        return false;
}
