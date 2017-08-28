// -*- c++ *--
// VideoSource_Linux_V4L.h
#ifndef __VideoSource_Linux_V4L_H
#define __VideoSource_Linux_V4L_H

#include "VideoSource.h"

class VideoSourceV4L : public VideoSource
{
public:
    VideoSourceV4L();
    ~VideoSourceV4L(){}
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);

private:
    void *mptr;
};
#endif
