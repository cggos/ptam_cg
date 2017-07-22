#!/bin/sh

sudo apt-get install gfortran libtiff5 libtiff-dev libpng++-dev libpng-dev libjpeg-dev libglu1-mesa-dev libgl1-mesa-dev libx11-dev libdc1394-22 libdc1394-22-dev libncurses5-dev libgfortran-5-dev freeglut3 freeglut3-dev libreadline6 libreadline6-dev libsuitesparse-dev liblapack-dev
sudo apt-get install libqt4-dev qt5-default

PTAM3rdParty=3rdParty
mkdir ${PTAM3rdParty}
cd ${PTAM3rdParty}
Path3rdParty=`pwd`

wget http://www.netlib.org/lapack/lapack-3.6.0.tgz
tar xvzf lapack-3.6.0.tgz
cd lapack-3.6.0
cp make.inc.example make.inc
make all -j $(nproc)
sudo cp *.a /usr/local/lib/

cd ${Path3rdParty}
wget https://www.edwardrosten.com/cvd/TooN-2.2.tar.xz
tar xvJf TooN-2.2.tar.xz 
cd TooN-2.2
./configure && make && sudo make install

cd ${Path3rdParty}
wget https://www.edwardrosten.com/cvd/libcvd-20150407.tar.xz
tar xvJf libcvd-20150407.tar.xz
cd libcvd-20150407
./configure && make && sudo make install

cd ${Path3rdParty}
wget https://www.edwardrosten.com/cvd/gvars-3.0.tar.xz
tar xvJf gvars-3.0.tar.xz
cd gvars-3.0
./configure && make && sudo make install

cd ${Path3rdParty}
git clone git@github.com:RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build && cmake ../ && make -j$(nproc) && sudo make install

sudo ldconfig
