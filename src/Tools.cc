#include "Tools.h"

namespace cg
{

    void Tools::LHFromRH(SE3<> rhm,SE3<> &lhm)
    {
        Matrix<3,3,float> rot = rhm.get_rotation().get_matrix();
        Vector<3,float> trans = rhm.get_translation();

        Matrix<3,3,float> rotation = rot;
        Vector<3,float> translation = trans;
        rotation[2][0] *= -1;
        rotation[2][1] *= -1;
        rotation[0][2] *= -1;
        rotation[1][2] *= -1;
        translation[2] *= -1;
        //compose SE3
        lhm.get_rotation() = rotation;
        lhm.get_translation() = translation;
    }

    void Tools::SE32Array(SE3<> se3, float* array)
    {
        Matrix<3,3,double> rot = se3.get_rotation().get_matrix();
        Vector<3,double> trans = se3.get_translation();

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                array[i*4+j] = (float)rot(i,j); // to row-major matrix

        array[3]  = (float)trans[0];
        array[7]  = (float)trans[1];
        array[11] = (float)trans[2];
    }

    void Tools::SO32RM(SO3<> so3, float* rm)
    {
        Matrix<3,3,double> rot = so3.get_matrix();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rm[i*3+j] = (float)rot(i,j); // to row-major matrix
    }

    void Tools::RM2SO3(float* rm, SO3<> &so3)
    {
        Matrix<3,3,double> rot;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rot[i][j] = rm[i*3+j];
        so3 = rot;
    }

/*
 * Ref        : http://www.cnblogs.com/wqj1212/archive/2010/11/21/1883033.html
 * Constraint : Cartesion Coordinate System
 * quater[4]  : x, y, z, w
 * *euler[3]  : theta_x, theta_y, theta_z
 */
    void Tools::Quaternion2Euler(float *quater, float **euler)
    {
        float x = quater[0];
        float y = quater[1];
        float z = quater[2];
        float w = quater[3];
        (*euler)[0] = atan2 ( 2*(w*x+y*z), 1-2*(x*x+y*y) );
        (*euler)[1] = asin  ( 2*(w*y-z*x) );
        (*euler)[2] = atan2 ( 2*(w*z+x*y), 1-2*(y*y+z*z) );
    }

    void Tools::Euler2Quaternion(float *euler, float **quater)
    {
        // Abbreviations for the various angular functions
        float cy = cos(euler[2] * 0.5);
        float sy = sin(euler[2] * 0.5);
        float cr = cos(euler[0] * 0.5);
        float sr = sin(euler[0] * 0.5);
        float cp = cos(euler[1] * 0.5);
        float sp = sin(euler[1] * 0.5);

        (*quater)[3] = cy * cr * cp + sy * sr * sp;
        (*quater)[0] = cy * sr * cp - sy * cr * sp;
        (*quater)[1] = cy * cr * sp + sy * sr * cp;
        (*quater)[2] = sy * cr * cp - cy * sr * sp;
    }

    /*
     * Ref       : https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-roational-matrix
     *     rm[9] : x, y, z, w
     * *euler[3] : theta_x, theta_y, theta_z
     * R = Rz(φ)Ry(θ)Rx(ψ)
     */
    void Tools::RotationMatrix2Euler(float *rm, float **euler)
    {
        float rm11 = rm[0];
        float rm21 = rm[3];
        float rm31 = rm[6];
        float rm32 = rm[7];
        float rm33 = rm[8];
        (*euler)[0] = atan2( rm32 , rm33 );
        (*euler)[1] = atan2(-rm31 , sqrt(rm32*rm32+rm33*rm33) );
        (*euler)[2] = atan2( rm21 , rm11 );

        (*euler)[0] = (*euler)[0] ;//* 180.0 / 3.1415926;
        (*euler)[1] = (*euler)[1] ;//* 180.0 / 3.1415926;
        (*euler)[2] = (*euler)[2] ;//* 180.0 / 3.1415926;
    }

    void Tools::GetCenterOfMass3D(const std::vector<Vector<3> > &vvPts, Vector<3> &center)
    {
        unsigned int nSize = vvPts.size();
        if(nSize < 1)
            return;
        for(unsigned int i=0; i<nSize; i++)
        {
            center += vvPts[i];
        }
        center /= nSize;
    }

//ICP using Sigular Values Decomposition
    void Tools::ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB)
    {
        unsigned int nSizeA = ptsA.size();
        unsigned int nSizeB = ptsB.size();

        if(nSizeA==0 || nSizeA != nSizeB)
            return;

        // center of mass
        Vector<3> ptCenterA = TooN::Zeros;
        Vector<3> ptCenterB = TooN::Zeros;
        GetCenterOfMass3D(ptsA,ptCenterA);
        GetCenterOfMass3D(ptsB,ptCenterB);

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
        GetCenterOfMass3D(ptsA,ptCenterA);
        GetCenterOfMass3D(ptsB,ptCenterB);

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
//
//    float as = 12.2717 /(ans[0]-(ans[0]+ans1[0])/2.0);
//    float as1 = 8.64864/(ans[1]-(ans[1]+ans1[1])/2.0);
//
//    std::cout << "as:\n" << as << " " << as1 << std::endl;
    }

}

