// -*- c++ *--
// VideoSource_Linux_DataSet.h
//
// Modified by GaoHongchen on 2017.08.01
//
#ifndef __VideoSource_DataSet_H
#define __VideoSource_DataSet_H

#include <string>
#include <fstream>

#include "VideoSource.h"

class VideoSourceDataSet : public VideoSource
{
public:
    VideoSourceDataSet();
    ~VideoSourceDataSet();
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);

private:
    std::string mDatasetPath;
    std::ifstream mFileIn;
    unsigned int mIndexImg;
};
#endif
