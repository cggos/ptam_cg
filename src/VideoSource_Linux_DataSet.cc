// Copyright 2017 by GaoHongchen
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/image_io.h>
#include <cvd/exceptions.h>

#include <unistd.h>
#include <exception>
#include <iomanip>

#include "VideoSource_Linux_DataSet.h"

VideoSourceDataSet::VideoSourceDataSet():mDatasetPath(""),mIndexImg(0)
{
    cout << "VideoSource_Linux: Opening RGB Image DataSet..." << endl;

    mDatasetPath = GV3::get<string>("VideoSource.DataSet", "./data/rgbd_dataset_freiburg1_xyz");
    cout << "VideoSource.DataSet: " << mDatasetPath << endl;

    std::string rgb_txt = mDatasetPath + "/rgb.txt";
    mFileIn.open(rgb_txt.c_str(),ios_base::in);
    if (!mFileIn.is_open())
    {
        std::cerr<<"VideoSource_Linux: cann't find rgb.txt!"<<std::endl;
        return;
    }

    cout << "VideoSource_Linux: Got RGB Image DataSet." << endl;
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
        mFileIn.seekg(0,ios_base::beg);
        mIndexImg = 0;
    }

    // 25 FPS (Frame per second)
    usleep(40000);//delay 40 milliseconds

    if(mIndexImg == 0)
    {
        string rgb_line;
        std::cout << "\nwhile loop begin" << std::endl;
        getline(mFileIn,rgb_line);
        while(rgb_line.find('#')!=std::string::npos)
        {
            std::cout << rgb_line << std::endl;
            getline(mFileIn,rgb_line);
        }
        std::cout << "while loop end\n" << std::endl;
    }

    std::string rgb_file, time_rgb;
    mFileIn >> time_rgb >> rgb_file;
    if(mFileIn.fail())//file end
    {
        goto FILE_END;
    }
    std::cout << "Img Index: " << setfill('0') << setw(3) << ++mIndexImg << ", time_rgb rgb_file: " << time_rgb << " " << rgb_file << std::endl;
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
        cout<<"GetAndFillFrameBWandRGB: std::exception-->\n"<< e.what() <<endl;
        GUI.ParseLine("quit");
    }
    catch(CVD::Exceptions::Image::All &e)
    {
        cout<<"GetAndFillFrameBWandRGB: CVD::Exceptions::Image::All-->\n"<< e.what <<endl;
        GUI.ParseLine("quit");
    }
    catch(...)
    {
        cout<<"===================================\n"<<endl;
        cout<<"GetAndFillFrameBWandRGB: exception!\n"<<endl;
        cout<<"===================================\n"<<endl;
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
