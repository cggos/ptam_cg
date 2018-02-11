// Copyright 2017 by GaoHongchen

#include "VideoSource.h"

#include <unistd.h>
#include <exception>
#include <iomanip>

#include <cvd/Linux/dvbuffer3.h>
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/image_io.h>
#include <cvd/exceptions.h>

using namespace CVD;
using namespace GVars3;

VideoSourceDV::VideoSourceDV()
{
    std::cout << "  VideoSourceDV: Opening video source..." << std::endl;
    DVBuffer3<yuv411>* pvb= new DVBuffer3<yuv411>();
    mptr = pvb;
    mirSize = pvb->size();
    std::cout << "  VideoSourceDV: Got video source." << std::endl;
}

void VideoSourceDV::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
    DVBuffer3<yuv411>* pvb = (DVBuffer3<yuv411>*) mptr;
    VideoFrame<yuv411> *pVidFrame = pvb->get_frame();
    convert_image(*pVidFrame, imBW);
    convert_image(*pVidFrame, imRGB);
    pvb->put_frame(pVidFrame);
}


VideoSourceV4L::VideoSourceV4L()
{
    std::cout << "  VideoSourceV4L: Opening video source..." << std::endl;
    std::string QuickCamFile = GV3::get<std::string>("VideoSource.V4LDevice", "/dev/video0");
    int nFrameRate = GV3::get<int>("VideoSource.Framerate", 30);
    V4LBuffer<yuv422>* pvb = new V4LBuffer<yuv422>(QuickCamFile, mirSize, -1, false, nFrameRate);
    mirSize = pvb->size();
    mptr = pvb;
    std::cout << "  VideoSourceV4L: Got video source." << std::endl;
}

void VideoSourceV4L::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
    V4LBuffer<yuv422>* pvb = (V4LBuffer<yuv422>*) mptr;
    VideoFrame<yuv422> *pVidFrame = pvb->get_frame();
    convert_image(*pVidFrame, imBW);
    convert_image(*pVidFrame, imRGB);
    pvb->put_frame(pVidFrame);
}


VideoSourceDataSet::VideoSourceDataSet():mDatasetPath(""),mIndexImg(0)
{
    std::cout << "VideoSource_Linux: Opening RGB Image DataSet..." << std::endl;

    mDatasetPath = GV3::get<std::string>("VideoSource.DataSet", "./data/rgbd_dataset_freiburg1_xyz");
    std::cout << "VideoSource.DataSet: " << mDatasetPath << std::endl;

    std::string rgb_txt = mDatasetPath + "/rgb.txt";
    mFileIn.open(rgb_txt.c_str(),std::ios_base::in);
    if (!mFileIn.is_open())
    {
        std::cerr<<"VideoSource_Linux: cann't find rgb.txt!"<<std::endl;
        return;
    }

    std::cout << "VideoSource_Linux: Got RGB Image DataSet." << std::endl;
}

void VideoSourceDataSet::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
    if (!mFileIn.is_open())
    {
        std::cerr<<"GetAndFillFrameBWandRGB: cann't find rgb.txt!"<<std::endl;
        GUI.ParseLine("quit");
        return;
    }
    if(mFileIn.fail())
    {
        mFileIn.close();
        GUI.ParseLine("quit");
        return;
    }
FILE_END:
    if(mFileIn.eof())
    {
        std::cout<<"GetAndFillFrameBWandRGB: mFileIn file end!\n"<<std::endl;

        std::cout<<"GetAndFillFrameBWandRGB: read mFileIn file again!"<<std::endl;
        mFileIn.clear();// very important !!!
        mFileIn.seekg(0,std::ios_base::beg);
        mIndexImg = 0;
    }

    // 25 FPS (Frame per second)
    usleep(40000);//delay 40 milliseconds

    if(mIndexImg == 0)
    {
        std::string rgb_line;
        std::getline(mFileIn,rgb_line);
        while(rgb_line.find('#')!=std::string::npos)
        {
            std::cout << rgb_line << std::endl;
            std::getline(mFileIn,rgb_line);
        }
    }

    std::string rgb_file, time_rgb;
    mFileIn >> time_rgb >> rgb_file;
    if(mFileIn.fail())//file end
    {
        goto FILE_END;
    }
    //std::cout << "Img Index: " << std::setfill('0') << std::setw(3) << ++mIndexImg << ", time_rgb rgb_file: " << time_rgb << " " << rgb_file << std::endl;
    try
    {
        std::string rgb_path = mDatasetPath+"/"+rgb_file;
        Image<Rgb<byte> > rgb_img = img_load( rgb_path );

        convert_image(rgb_img, imBW);
        convert_image(rgb_img, imRGB);

//        img_save(imBW,"imBW.bmp");
//        img_save(imRGB,"imRGB.bmp");
    }
    catch(std::exception &e)
    {
        std::cout<<"GetAndFillFrameBWandRGB: std::exception-->\n"<< e.what() << std::endl;
        GUI.ParseLine("quit");
    }
    catch(CVD::Exceptions::Image::All &e)
    {
        std::cout<<"GetAndFillFrameBWandRGB: CVD::Exceptions::Image::All-->\n"<< e.what << std::endl;
        GUI.ParseLine("quit");
    }
    catch(...)
    {
        std::cout<<"===================================\n"<<std::endl;
        std::cout<<"GetAndFillFrameBWandRGB: exception!\n"<<std::endl;
        std::cout<<"===================================\n"<<std::endl;
        GUI.ParseLine("quit");
    }
}

VideoSourceDataSet::~VideoSourceDataSet()
{
    if (mFileIn.is_open())
    {
        mFileIn.close();
    }
}
