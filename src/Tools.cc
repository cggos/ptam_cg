#include "Tools.h"
#include <iostream>

//ICP using Sigular Values Decomposition
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

// ICP using Quarternion Decomposition
void Tools::ICP_QD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB)
{
    unsigned int nSizeA = ptsA.size();
    unsigned int nSizeB = ptsB.size();

    if(nSizeA==0 || nSizeA != nSizeB)
        return;

    // center of mass
    Vector<3> ptCenterA = TooN::Zeros;
    Vector<3> ptCenterB = TooN::Zeros;
    for(unsigned int i=0; i<nSizeA; i++)
    {
        ptCenterA += ptsA[i];
        ptCenterB += ptsB[i];
    }
    ptCenterA /= nSizeA;
    ptCenterB /= nSizeB;

    // remove center
    std::vector<Vector<3> > ptsANew;
    std::vector<Vector<3> > ptsBNew;
    ptsANew.reserve(nSizeA);
    ptsBNew.reserve(nSizeB);
    for(unsigned int i=0;i<nSizeA;i++)
    {
        Vector<3> v3PtA = ptsA[i] - ptCenterA;
        Vector<3> v3PtB = ptsB[i] - ptCenterB;
        ptsANew.push_back(v3PtA);
        ptsBNew.push_back(v3PtB);
    }

    // Get Quaternion
    Matrix<3, Dynamic> MA(3, nSizeA);
    Matrix<3, Dynamic> MB(3, nSizeB);
    for(unsigned int i=0;i<nSizeA;i++)
    {
        MA.T()[i] = ptsANew[i];
        MB.T()[i] = ptsBNew[i];
    }
    Matrix<3> M = MB * MA.T();
    M /= nSizeA;

    Vector<3> vDelta = TooN::Zeros;
    Matrix<3> skewsymM = M - M.T();
    vDelta[0] = skewsymM(1,2);
    vDelta[1] = skewsymM(2,0);
    vDelta[2] = skewsymM(0,1);

    Matrix<4> Q = TooN::Identity;
    Q(0,0) = TooN::trace(M);
    Q.slice<0,1,1,3>() = vDelta.as_row();
    Q.slice<1,0,3,1>() = vDelta.as_col();
    Q.slice<1,1,3,3>() = M + M.T() - TooN::trace(M)*TooN::Identity;

    SymEigen<4> eigM(Q);
    float eigenvalueMax = eigM.get_evalues()[3];
    SVD<4> svd(Q-eigenvalueMax*TooN::Identity);
    Vector<4> v4Quarter = -svd.get_VT()[3];

    // Quarternion to Rotation Matrix
    Matrix<3> RM = TooN::Identity;
    RM(0,0) = std::pow(v4Quarter[0],2) + std::pow(v4Quarter[1],2) - std::pow(v4Quarter[2],2) - std::pow(v4Quarter[3],2);
    RM(0,1) = 2*(v4Quarter[1]*v4Quarter[2]-v4Quarter[0]*v4Quarter[3]);
    RM(0,2) = 2*(v4Quarter[1]*v4Quarter[3]+v4Quarter[0]*v4Quarter[2]);
    RM(1,0) = 2*(v4Quarter[1]*v4Quarter[2]+v4Quarter[0]*v4Quarter[3]);
    RM(1,1) = std::pow(v4Quarter[0],2) - std::pow(v4Quarter[1],2) + std::pow(v4Quarter[2],2) - std::pow(v4Quarter[3],2);
    RM(1,2) = 2*(v4Quarter[2]*v4Quarter[3]-v4Quarter[0]*v4Quarter[1]);
    RM(2,0) = 2*(v4Quarter[1]*v4Quarter[3]-v4Quarter[0]*v4Quarter[2]);
    RM(2,1) = 2*(v4Quarter[2]*v4Quarter[3]+v4Quarter[0]*v4Quarter[1]);
    RM(2,2) = std::pow(v4Quarter[0],2) - std::pow(v4Quarter[1],2) - std::pow(v4Quarter[2],2) + std::pow(v4Quarter[3],2);

    // Compose SE3
    SO3<> so3 = RM;
    Vector<3> translateAB = ptCenterA-ptCenterB;

    se3AfromB.get_rotation() = so3;
    se3AfromB.get_translation() = translateAB;

//    Vector<3> ans = so3*ptsB[0]+translateAB;
//    Vector<3> ans1 = so3*ptsB[2]+translateAB;
//    std::cout << "ans:\n" << ans << " " << ans1 << std::endl;

//    float as = 12.2717 /(ans[0]-(ans[0]+ans1[0])/2.0);
//    float as1 = 8.64864/(ans[1]-(ans[1]+ans1[1])/2.0);

//    std::cout << "as:\n" << as << " " << as1 << std::endl;
}
