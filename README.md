# PTAM4AR

[![Join the chat at https://gitter.im/PTAM4AR/Lobby](https://badges.gitter.im/PTAM4AR/Lobby.svg)](https://gitter.im/PTAM4AR/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) [![Documentation](https://codedocs.xyz/GaoHongchen/PTAM4AR.svg)](https://codedocs.xyz/GaoHongchen/PTAM4AR/)  
PTAM source code for AR based on the one from Isis.

# Introduction
PTAM (Parallel Tracking and Mapping) is a camera tracking system for augmented reality.

# Building

## Build Status on CI
* Linux (Ubuntu-14.04 64 bits, GCC-5) : [![Build Status](https://travis-ci.org/GaoHongchen/PTAM4AR.svg?branch=master)](https://travis-ci.org/GaoHongchen/PTAM4AR) [![Coverage Status](https://coveralls.io/repos/github/GaoHongchen/PTAM4AR/badge.svg?branch=master)](https://coveralls.io/github/GaoHongchen/PTAM4AR?branch=master)

## Build This Project locally

**1) Install dependencies**
* Execute the script to install dependencies or 3rdParties on Linux: [3rdParty_install.sh](./scripts/3rdParty_install.sh)   

**2) Build Project**
* Execute the script: [build_project.sh](./scripts/build_project.sh)

## Relate Compilation for PTAM
* [PTAM Compilation on Linux](http://hustcalm.me/blog/2013/09/27/ptam-compilation-on-linux-howto/)
* [Build PTAM on Ubuntu 11.10](http://irawiki.disco.unimib.it/irawiki/index.php/PTAM)
* [Installation of PTAM in ROS](https://sites.google.com/site/zhilongliuwebsite/research/computer-vision-embedded-systems/ptam)

# Tools

## Static Analysis of C/C++
* cppcheck: [Add cppcheck and clang-format for a cmake project](https://arcanis.me/en/2015/10/17/cppcheck-and-clang-format)

## Test Code Coverage
* coveralls-cmake

## Generate API documentation
* Doxygen

# Relate Source Code

## Linux
* [PTAM-GPL (GitHub)](https://github.com/Oxford-PTAM/PTAM-GPL)
* [PTAM-linux-cv2.3 (GitHub)](https://github.com/nttputus/PTAM-linux-cv2.3)

## Android
* [APTAM-GPL](https://github.com/ICGJKU/APTAM-GPL)
* [android-ptam](https://github.com/damienfir/android-ptam)

## Windows
* [PTAM-Windows](https://github.com/LucRyan/PTAM-Windows)

# PTAM Tutorials
* [PTAM Official Site](http://www.robots.ox.ac.uk/~gk/PTAM/)
* [PTAM news](https://ewokrampage.wordpress.com/)
* [PTAM-Monocular SLAM](http://www.doc.ic.ac.uk/~gj414/monocular_slam/ptam.html)
* [implementing-ptam-stereo-tracking-and-pose-estimation-for-ar-with-opencv-w-code](http://www.morethantechnical.com/2010/03/06/implementing-ptam-stereo-tracking-and-pose-estimation-for-ar-with-opencv-w-code/)
* [PTAM on ROS](http://wiki.ros.org/ptam)
