#ifndef __Image_Process_H
#define __Image_Process_H

#include <cvd/image.h>
#include <cvd/byte.h>

class ImageProcess
{
public:
    ImageProcess() {}
    static double FindShiTomasiScoreAtPoint(CVD::BasicImage<CVD::byte> &image, int nHalfBoxSize, CVD::ImageRef irCenter);
};

#endif
