// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//
// VideoSource.h
// Declares the VideoSource class
//
// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.
//
#ifndef __VideoSource_H
#define __VideoSource_H

#include <string>
#include <fstream>

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <gvars3/instances.h>

class VideoSource
{
public:
    VideoSource():mirSize(GVars3::GV3::get<CVD::ImageRef>("VideoSource.Resolution", CVD::ImageRef(640,480))){}
    virtual ~VideoSource(){}
    virtual void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB)=0;
    CVD::ImageRef Size() { return mirSize; }
protected:
    CVD::ImageRef mirSize;
};


class VideoSourceDV : public VideoSource
{
public:
    VideoSourceDV();
    ~VideoSourceDV(){}
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);

private:
    void *mptr;
};


class VideoSourceV4L : public VideoSource
{
public:
    VideoSourceV4L();
    ~VideoSourceV4L(){}
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);

private:
    void *mptr;
};


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
