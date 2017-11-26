#ifndef __TOOLS_H
#define __TOOLS_H

#include <cassert>
#include <vector>
#include <cmath>

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

using namespace TooN;

namespace cg
{

class Tools
{
public:
    static void LHFromRH(SE3<> rhm,SE3<> &lhm);

    static void SE32Array(SE3<> se3, float* array);
    static void SO32RM(SO3<> so3, float* rm);
    static void RM2SO3(float* rm, SO3<> &so3);

    static void Quaternion2Euler(float *quater, float **euler);
    static void Euler2Quaternion(float *euler, float **quater);
    static void RotationMatrix2Euler(float *rm, float **euler);

    static void GetCenterOfMass3D(const std::vector<Vector<3> > &vvPts, Vector<3> &center);
    static void ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
    static void ICP_QD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);

    // Inverse of 2-matrix
    // Must be invertible!
    inline static Matrix<2> M2Inverse(const Matrix<2> &m)
    {
        Matrix<2> m2Res;
        double dDet = m[0][0] * m[1][1] - m[1][0] * m[0][1];
        assert(dDet!=0.0);
        double dInverseDet = 1.0 / dDet;
        m2Res[0][0] = m[1][1] * dInverseDet;
        m2Res[1][1] = m[0][0] * dInverseDet;
        m2Res[1][0] = -m[1][0] * dInverseDet;
        m2Res[0][1] = -m[0][1] * dInverseDet;
        return m2Res;
    }

    // Determinant of 2x2
    inline static double M2Det(Matrix<2> m)
    {
        return m[0][0] * m[1][1]  - m[0][1] * m[1][0];
    }

    // Determinant of 3x3
    inline static double M3Det(Matrix<3> m )
    {
        return
                m[0][0] * (m[1][1] * m[2][2]  - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2]  - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1]  - m[1][1] * m[2][0]);
    }
};

}

#endif
