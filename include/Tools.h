#ifndef __TOOLS_H
#define __TOOLS_H

#include <cassert>
#include <vector>
#include <cmath>
#include <algorithm>

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
        struct Quat
        {
            float x;
            float y;
            float z;
            float w;
            Quat():x(0),y(0),z(0),w(1){}
        };

        static void LHFromRH(SE3<> rhm,SE3<> &lhm);

        static void SE32Array(SE3<> se3, float* array);
        static void Array2SE3(const float* array, SE3<> &se3);

        static void SO32Array(SO3<> so3, float* array);
        static void Array2SO3(const float* array, SO3<> &so3);

        static void Quaternion2SO3(Quat q, SO3<> &so3);
        static void Quaternion2SO3_matlab(Quat q, SO3<> &so3);

        static void SO32Quaternion(SO3<> so3, Quat &q);

        static void Quaternion2Euler(const float *quater, float **euler);
        static void Euler2Quaternion(const float *euler,  float **quater);

        static void RotationMatrix2Euler(const float *rm, float **euler);

        static void GetCenterOfMass3D(const std::vector<Vector<3> > &vvPts, Vector<3> &center);
        static void ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
        static void ICP_QD( const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);

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


    /**
     * @brief Defines various MEstimators which can be used by the Tracker and the Bundle adjuster.
     *        Not that some of the inputs are square quantities!
     */
    class MEstimator
    {
    protected:
        static double FindSigmaSquared(std::vector<double> &vdErrorSquared){ return -1.0;};
        static double SquareRootWeight(double dErrorSquared, double dSigmaSquared){ return -1.0;};
        static double Weight(double dErrorSquared, double dSigmaSquared){ return -1.0;};
        static double ObjectiveScore(double dErrorSquared, double dSigmaSquared){ return -1.0;};
    };
    struct Tukey : public MEstimator
    {
        inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
        inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
        inline static double Weight(double dErrorSquared, double dSigmaSquared);
        inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
    };
    struct Cauchy : public MEstimator
    {
        inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
        inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
        inline static double Weight(double dErrorSquared, double dSigmaSquared);
        inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
    };

    struct Huber : public MEstimator
    {
        inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
        inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
        inline static double Weight(double dErrorSquared, double dSigmaSquared);
        inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
    };

    struct LeastSquares : public MEstimator
    {
        inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
        inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
        inline static double Weight(double dErrorSquared, double dSigmaSquared);
        inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
    };

    ////////////////////////// Tukey //////////////////////////
    inline double Tukey::Weight(double dErrorSquared, double dSigmaSquared)
    {
        double dSqrt = SquareRootWeight(dErrorSquared, dSigmaSquared);
        return dSqrt * dSqrt;
    }

    inline double Tukey::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
    {
        if(dErrorSquared > dSigmaSquared)
            return 0.0;
        else
            return 1.0 - (dErrorSquared / dSigmaSquared);
    }

    inline double Tukey::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
    {
        // NB All returned are scaled because
        // I'm not multiplying by sigmasquared/6.0
        if(dErrorSquared > dSigmaSquared)
            return 1.0;
        double d = 1.0 - dErrorSquared / dSigmaSquared;
        return (1.0 - d*d*d);
    }

    inline double Tukey::FindSigmaSquared(std::vector<double> &vdErrorSquared)
    {
        double dSigmaSquared;
        assert(vdErrorSquared.size() > 0);
        std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
        double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
        double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
        dSigma =  4.6851 * dSigma;
        dSigmaSquared = dSigma * dSigma;
        return dSigmaSquared;
    }

    ////////////////////////// Cauchy //////////////////////////
    inline double Cauchy::Weight(double dErrorSquared, double dSigmaSquared)
    {
        return 1.0 / (1.0 + dErrorSquared / dSigmaSquared);
    }

    inline double Cauchy::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
    {
        return sqrt(Weight(dErrorSquared, dSigmaSquared));
    }

    inline double Cauchy::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
    {
        return log(1.0 + dErrorSquared / dSigmaSquared);
    }

    inline double Cauchy::FindSigmaSquared(std::vector<double> &vdErrorSquared)
    {
        double dSigmaSquared;
        assert(vdErrorSquared.size() > 0);
        std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
        double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
        double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
        dSigma =  4.6851 * dSigma;
        dSigmaSquared = dSigma * dSigma;
        return dSigmaSquared;
    }

    ////////////////////////// Huber //////////////////////////
    inline double Huber::Weight(double dErrorSquared, double dSigmaSquared)
    {
        if(dErrorSquared < dSigmaSquared)
            return 1;
        else
            return sqrt(dSigmaSquared / dErrorSquared);
    }

    inline double Huber::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
    {
        return sqrt(Weight(dErrorSquared, dSigmaSquared));
    }

    inline double Huber::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
    {
        if(dErrorSquared< dSigmaSquared)
            return 0.5 * dErrorSquared;
        else
        {
            double dSigma = sqrt(dSigmaSquared);
            double dError = sqrt(dErrorSquared);
            return dSigma * ( dError - 0.5 * dSigma);
        }
    }

    inline double Huber::FindSigmaSquared(std::vector<double> &vdErrorSquared)
    {
        double dSigmaSquared;
        assert(vdErrorSquared.size() > 0);
        std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
        double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
        double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
        dSigma =  1.345 * dSigma;
        dSigmaSquared = dSigma * dSigma;
        return dSigmaSquared;
    }

    ////////////////////////// LeastSquares //////////////////////////
    inline double LeastSquares::Weight(double dErrorSquared, double dSigmaSquared)
    {
        return 1.0;
    }

    inline double LeastSquares::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
    {
        return 1.0;
    }

    inline double LeastSquares::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
    {
        return dErrorSquared;
    }

    inline double LeastSquares::FindSigmaSquared(std::vector<double> &vdErrorSquared)
    {
        if(vdErrorSquared.size() == 0)
            return 0.0;
        double dSum = 0.0;
        for(unsigned int i=0; i<vdErrorSquared.size(); i++)
            dSum+=vdErrorSquared[i];
        return dSum / vdErrorSquared.size();
    }

}

#endif
