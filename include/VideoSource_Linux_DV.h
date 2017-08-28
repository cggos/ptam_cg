// -*- c++ *--
// VideoSource_Linux_DV.h
#ifndef __VideoSource_Linux_DV_H
#define __VideoSource_Linux_DV_H

#include "VideoSource.h"

class VideoSourceDV : public VideoSource
{
public:
    VideoSourceDV();
    ~VideoSourceDV(){}
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);

private:
    void *mptr;
};
#endif
