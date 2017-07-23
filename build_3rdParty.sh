#!/bin/sh

# 安装 build工具
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config
# 更新C++库
sudo apt-get install libboost-dev libboost-doc
# gfortran
sudo apt-get install gfortran libgfortran-5-dev
# 安装线性代数的低级库
sudo apt-get install liblapack-dev libblas-dev
# 图像IO 和 摄像机驱动
sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libdc1394-22-dev libv4l-dev
# 视频IO， 编解码 和 视频显示库
sudo apt-get install libavcodec-dev libavformat-dev libavutil-dev libpostproc-dev libswscale-dev libavdevice-dev libsdl-dev
sudo apt-get install libgtk2.0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
# OpenGL
sudo apt-get install mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev
# qt4 & qt5
sudo apt-get install libqt4-dev qt5-default
# others
sudo apt-get install libx11-dev libncurses5-dev libreadline6 libreadline-dev libsuitesparse-dev

PTAM3rdParty=3rdParty
mkdir ${PTAM3rdParty}
cd ${PTAM3rdParty}
Path3rdParty=`pwd`

ulimit -s unlimited

# lapack
wget http://www.netlib.org/lapack/lapack-3.6.0.tgz
tar xvzf lapack-3.6.0.tgz
cd lapack-3.6.0
cp make.inc.example make.inc
make all -j $(nproc)
sudo cp *.a /usr/local/lib/

# TooN
cd ${Path3rdParty}
# sudo git clone git://github.com/edrosten/TooN.git
wget https://www.edwardrosten.com/cvd/TooN-2.2.tar.xz
tar xvJf TooN-2.2.tar.xz 
cd TooN-2.2
./configure && make && sudo make install

# libCVD
cd ${Path3rdParty}
# sudo git clone git://github.com/edrosten/libcvd.git
wget https://www.edwardrosten.com/cvd/libcvd-20150407.tar.xz
tar xvJf libcvd-20150407.tar.xz
cd libcvd-20150407
./configure && make && sudo make install

# GVars3
cd ${Path3rdParty}
# sudo git clone git://github.com/edrosten/gvars.git
wget https://www.edwardrosten.com/cvd/gvars-3.0.tar.xz
tar xvJf gvars-3.0.tar.xz
cd gvars-3.0
./configure && make && sudo make install

# G2O
cd ${Path3rdParty}
git clone git@github.com:RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build && cmake ../ && make -j$(nproc) && sudo make install

sudo ldconfig

exit 0
