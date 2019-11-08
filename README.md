# ptam_cg

[![Join the chat at https://gitter.im/ptam_cg_01/Lobby](https://badges.gitter.im/ptam_cg_01/Lobby.svg)](https://gitter.im/ptam_cg_01/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) [![Documentation](https://codedocs.xyz/GaoHongchen/PTAM4AR.svg)](https://codedocs.xyz/GaoHongchen/PTAM4AR/) [![Build Status](https://travis-ci.org/cggos/ptam_cg.svg?branch=master)](https://travis-ci.org/cggos/ptam_cg) [![Coverage Status](https://coveralls.io/repos/github/cggos/ptam_cg/badge.svg?branch=master)](https://coveralls.io/github/cggos/ptam_cg?branch=master)

Modified PTAM source code for AR based on [the one from Isis](http://www.robots.ox.ac.uk/~gk/PTAM).

**PTAM (Parallel Tracking and Mapping)** is a camera tracking system for augmented reality.

My Blogs:
* [SLAM之PTAM学习笔记](https://blog.csdn.net/u011178262/article/details/79315782)

-----


# Build

* **Build locally**
  - Dependencies: `bash install_deps.sh`
  - Project: `mkdir build && cd build && cmake .. && time make -j2`

* **Related Compilation for PTAM**
  - [PTAM Compilation on Linux](http://hustcalm.me/blog/2013/09/27/ptam-compilation-on-linux-howto/)
  - [Build PTAM on Ubuntu 11.10](http://irawiki.disco.unimib.it/irawiki/index.php/PTAM)


# Related Source Code

* **Linux**
  - [PTAM-GPL (GitHub)](https://github.com/Oxford-PTAM/PTAM-GPL)
  - [PTAM-linux-cv2.3 (GitHub)](https://github.com/nttputus/PTAM-linux-cv2.3)
  
* **Python**
  - [uoip/stereo_ptam](https://github.com/uoip/stereo_ptam)

* **ROS**
  - [ethzasl_ptam (ROS Wiki)](http://wiki.ros.org/ptam)
  - [ethzasl_ptam (GitHub)](https://github.com/ethz-asl/ethzasl_ptam)
    - [Installation of PTAM in ROS](https://sites.google.com/site/zhilongliuwebsite/research/computer-vision-embedded-systems/ptam)
    - issue **[Build on kinetic #98](https://github.com/ethz-asl/ethzasl_ptam/pull/98/files)**

* **Android**
  - [APTAM-GPL](https://github.com/ICGJKU/APTAM-GPL)
  - [android-ptam](https://github.com/damienfir/android-ptam)

* **Windows**
  - [PTAM-Windows](https://github.com/LucRyan/PTAM-Windows)


# PTAM Tutorials

* [PTAM news](https://ewokrampage.wordpress.com/)
* [implementing-ptam-stereo-tracking-and-pose-estimation-for-ar-with-opencv-w-code](http://www.morethantechnical.com/2010/03/06/implementing-ptam-stereo-tracking-and-pose-estimation-for-ar-with-opencv-w-code/)
* [窥探PTAM (CSDN)](https://blog.csdn.net/ilotuo/article/category/6297333)
* [PTAM跟踪过程中的旋转预测方法](https://zhuanlan.zhihu.com/p/20302059)
