#!/usr/bin/env bash

# Project directory
PTAMDir=`pwd`

sudo apt-get update
# Base tools
sudo apt-get install libx11-dev libncurses5-dev libreadline6 libreadline-dev
# Build tools
sudo apt-get install build-essential cmake pkg-config
# Boost for C++
sudo apt-get install libboost-dev
# gfortran
sudo apt-get install gfortran libgfortran-5-dev
# Low level libraries for Linear Algebra
sudo apt-get install liblapack-dev libblas-dev libsuitesparse-dev
# Image I/O && Camera Driver
sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libdc1394-22-dev libv4l-dev
# Video I/O && Codec && Display
sudo apt-get install libavcodec-dev libavformat-dev libavutil-dev libpostproc-dev libswscale-dev libavdevice-dev libsdl-dev
sudo apt-get install libgtk2.0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
# OpenGL
sudo apt-get install mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev
# qt4 & qt5
sudo apt-get install libqt4-dev qt5-default

echo -e "\n Making 3rdParty directory in the current project directory..."
PTAM3rdParty=3rdParty
mkdir ${PTAM3rdParty}
cd ${PTAM3rdParty}
Path3rdParty=`pwd`

ulimit -s unlimited

# TooN
echo -e "\n Installing TooN... \n"
cd ${Path3rdParty}
# sudo git clone git://github.com/edrosten/TooN.git
wget https://www.edwardrosten.com/cvd/TooN-2.2.tar.xz
tar xvJf TooN-2.2.tar.xz
cd TooN-2.2
./configure && make && sudo make install

# libCVD
echo -e "\n Installing libCVD... \n"
cd ${Path3rdParty}
# sudo git clone git://github.com/edrosten/libcvd.git
wget https://www.edwardrosten.com/cvd/libcvd-20150407.tar.xz
tar xvJf libcvd-20150407.tar.xz
cd libcvd-20150407
./configure && make && sudo make install

# GVars3
echo -e "\n Installing GVars3... \n"
cd ${Path3rdParty}
# sudo git clone git://github.com/edrosten/gvars.git
wget https://www.edwardrosten.com/cvd/gvars-3.0.tar.xz
tar xvJf gvars-3.0.tar.xz
cd gvars-3.0
./configure && make && sudo make install

echo -e "\n Make the libs work \n"
sudo ldconfig

echo -e "\n Install 3rdParties successfully! \n"

cd ${PTAMDir}

exit 0
