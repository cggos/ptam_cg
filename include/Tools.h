#ifndef __TOOLS_H
#define __TOOLS_H

#include <vector>
#include <TooN/se3.h>
#include <TooN/SVD.h>
using namespace TooN;

class Tools
{
public:
    static void ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
};

#endif
