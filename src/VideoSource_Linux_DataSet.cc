// Copyright 2017 by GaoHongchen
#include "VideoSource_Linux_DataSet.h"
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/image_io.h>
#include <gvars3/instances.h>

#include <unistd.h>
#include <exception>

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource::VideoSource():mDatasetPath("")
{
    cout << "VideoSource_Linux: Opening RGB Image DataSet..." << endl;

    mDatasetPath = GV3::get<string>("VideoSource.DataSet", "./data/rgbd_dataset_freiburg1_xyz");
    cout << "VideoSource.DataSet: " << mDatasetPath << endl;

    mirSize = GV3::get<ImageRef>("VideoSource.Resolution", ImageRef(640,480));
    cout << "VideoSource.Resolution: " << mirSize << endl;

    std::string rgb_txt = mDatasetPath + "/rgb.txt";
    mFileIn.open(rgb_txt,ios_base::in);
    if (!mFileIn.is_open())
    {
        std::cerr<<"Cann't find rgb.txt!"<<std::endl;
        return;
    }

    cout << "VideoSource_Linux: Got RGB Image DataSet." << endl;
}

ImageRef VideoSource::Size()
{ 
    return mirSize;
}

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
    try
    {
        if(mFileIn.eof())
        {
            mFileIn.close();
            return;
        }
        std::string rgb_file, time_rgb;
        mFileIn >> time_rgb >> rgb_file;
        //getline(mFileIn,strLine);
        cout << "time_rgb: " << time_rgb << endl;
        cout << "rgb_file: " << rgb_file << endl;

        std::string rgb_path = mDatasetPath+"/"+rgb_file;
        Image<Rgb<byte> > rgb_img = img_load( rgb_path );

        convert_image(rgb_img, imBW);
        convert_image(rgb_img, imRGB);

        img_save(imBW,"imBW.bmp");
        img_save(imRGB,"imRGB.bmp");
    }
    catch(std::exception &e)
    {
        cout<<"GetAndFillFrameBWandRGB exception: "<< e.what() <<endl;
    }
}
