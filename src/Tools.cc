#include "Tools.h"

void Tools::ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB)
{
    unsigned int nSizeA = ptsA.size();
    unsigned int nSizeB = ptsB.size();

    if(nSizeA==0 || nSizeA != nSizeB)
        return;

    //center of mass
    Vector<3> ptCenterA, ptCenterB;
    for(unsigned int i=0; i<nSizeA; i++)
    {
        ptCenterA += ptsA[i];
        ptCenterB += ptsB[i];
    }
    ptCenterA /= nSizeA;
    ptCenterB /= nSizeB;

    //compute W
    Matrix<3> W;
    for(unsigned int i=0; i<nSizeA; i++)
    {
        W += (ptsA[i]-ptCenterA).as_col() * (ptsB[i]-ptCenterB).as_row();
    }

    int determinantW = W(0,0)*W(1,1)*W(2,2) + W(0,1)*W(1,2)*W(2,0) + W(0,2)*W(1,0)*W(2,1) -
            (W(0,0)*W(1,2)*W(2,1) + W(0,1)*W(1,0)*W(2,2) + W(0,2)*W(1,1)*W(2,0));
    assert(determinantW);

    //svd decomposition
    SVD<3,3> svd(W);
    SO3<> so3 = svd.get_U()*svd.get_VT();
    Vector<3> translation = ptCenterA-so3*ptCenterB;

    //compose SE3
    se3AfromB.get_rotation() = so3;
    se3AfromB.get_translation() = translation;
}
