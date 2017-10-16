#ifndef __TOOLS_H
#define __TOOLS_H

#include <vector>
#include <cmath>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>
using namespace TooN;

class Tools
{
public:
    static void ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
    static void ICP_QD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
};

#endif
