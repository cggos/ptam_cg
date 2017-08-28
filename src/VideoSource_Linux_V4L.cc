// Copyright 2008 Isis Innovation Limited
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>

#include "VideoSource_Linux_V4L.h"

VideoSourceV4L::VideoSourceV4L()
{
    cout << "  VideoSourceV4L: Opening video source..." << endl;
    string QuickCamFile = GV3::get<string>("VideoSource.V4LDevice", "/dev/video0");
    int nFrameRate = GV3::get<int>("VideoSource.Framerate", 30);
    V4LBuffer<yuv422>* pvb = new V4LBuffer<yuv422>(QuickCamFile, mirSize, -1, false, nFrameRate);
    mirSize = pvb->size();
    mptr = pvb;
    cout << "  VideoSourceV4L: Got video source." << endl;
}

void VideoSourceV4L::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
    V4LBuffer<yuv422>* pvb = (V4LBuffer<yuv422>*) mptr;
    VideoFrame<yuv422> *pVidFrame = pvb->get_frame();
    convert_image(*pVidFrame, imBW);
    convert_image(*pVidFrame, imRGB);
    pvb->put_frame(pVidFrame);
}
