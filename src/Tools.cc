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

    /**
     * @brief transform TooN::SE3 to array
     * @param se3 TooN::SE3 object
     * @param array array of 3x4 row-major matrix of RT
     */
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

    /**
     * @brief transform array to TooN::SE3
     * @param array array of 3x4 row-major matrix of RT
     * @param se3 TooN::SE3 object
     */
    void Tools::Array2SE3(const float *array, SE3<> &se3)
    {
        Matrix<3,3> m3Rotation;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                m3Rotation[i][j] = array[i*4+j];
            }
        }
        SO3<> so3 = SO3<>(m3Rotation);

        Vector<3> v3Translation;
        v3Translation[0] = array[ 3];
        v3Translation[1] = array[ 7];
        v3Translation[2] = array[11];

        se3.get_rotation()    = so3;
        se3.get_translation() = v3Translation;
    }

    /**
     * @brief transform TooN::SO3 to array
     * @param so3 TooN::SO3 object
     * @param array array of 3x3 row-major rotation matrix
     */
    void Tools::SO32Array(SO3<> so3, float* array)
    {
        Matrix<3,3,double> rot = so3.get_matrix();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                array[i*3+j] = (float)rot(i,j);
    }

    /**
     * @brief transform array to TooN::SO3
     * @param array array of 3x3 row-major rotation matrix
     * @param so3 TooN::SO3 object
     */
    void Tools::Array2SO3(const float* array, SO3<> &so3)
    {
        Matrix<3,3,double> rot;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rot[i][j] = array[i*3+j];
        so3 = rot;
    }

    /**
     * @brief transform Quaternion of Quat struct to TooN::SO3
     * @param q a Quaternion object
     * @param so3 TooN::SO3 object
     * @details the output so3 is the transpose or inverse of the one from matlab quat2dcm
     */
    void Tools::Quaternion2SO3(Quat q, SO3<> &so3)
    {
        float norm2QInv = 1.f / std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        q.x *= norm2QInv;
        q.y *= norm2QInv;
        q.z *= norm2QInv;
        q.w *= norm2QInv;

        float xx  = q.x * q.x;
        float xy  = q.x * q.y;
        float xz  = q.x * q.z;
        float xw  = q.x * q.w;

        float yy  = q.y * q.y;
        float yz  = q.y * q.z;
        float yw  = q.y * q.w;

        float zz  = q.z * q.z;
        float zw  = q.z * q.w;

        Matrix<3,3,float> m3R;
        m3R[0][0]  = 1 - 2 * ( yy + zz );
        m3R[0][1]  =     2 * ( xy - zw );
        m3R[0][2]  =     2 * ( xz + yw );

        m3R[1][0]  =     2 * ( xy + zw );
        m3R[1][1]  = 1 - 2 * ( xx + zz );
        m3R[1][2]  =     2 * ( yz - xw );

        m3R[2][0]  =     2 * ( xz - yw );
        m3R[2][1]  =     2 * ( yz + xw );
        m3R[2][2]  = 1 - 2 * ( xx + yy );

        so3 = SO3<>(m3R);
    }

    /**
     * @brief transform Quaternion of Quat struct to TooN::SO3
     * @param q a Quaternion object
     * @param so3 TooN::SO3 object
     * @details be same to quat2dcm of matlab
     */
    void Tools::Quaternion2SO3_matlab(Quat q, SO3<> &so3)
    {
        //normalize Q
        float norm2QInv = 1.f / std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        q.x *= norm2QInv;
        q.y *= norm2QInv;
        q.z *= norm2QInv;
        q.w *= norm2QInv;

        float xx  = q.x * q.x;
        float xy  = q.x * q.y;
        float xz  = q.x * q.z;
        float xw  = q.x * q.w;

        float yy  = q.y * q.y;
        float yz  = q.y * q.z;
        float yw  = q.y * q.w;

        float zz  = q.z * q.z;
        float zw  = q.z * q.w;

        float ww  = q.w * q.w;

        Matrix<3,3,float> m3R;

        m3R[0][0] = ww + xx - yy - zz;
        m3R[0][1] = 2 * (xy + zw);
        m3R[0][2] = 2 * (xz - yw);

        m3R[1][0] = 2 * (xy - zw);
        m3R[1][1] = ww - xx + yy - zz;
        m3R[1][2] = 2 * (yz + xw);

        m3R[2][0] = 2 * (xz + yw);
        m3R[2][1] = 2 * (yz - xw);
        m3R[2][2] = ww - xx - yy + zz;

        so3 = SO3<>(m3R);
    }

    /**
     * @brief transform TooN::SO3 to Quaternion of Quat struct
     * @param so3 TooN::SO3 object
     * @param q a Quaternion object
     * @details the output q is equal to [-x,-y,-z,w] where [x,y,z,w] is the output of matlab dcm2quat
     */
    void Tools::SO32Quaternion(SO3<> so3, Quat &q)
    {
        Matrix<3,3,float> rot = so3.get_matrix();
        float tr = rot(0,0) + rot(1,1) + rot(2,2);

        if (tr > 0) {
            float S = std::sqrt(tr+1.0f) * 2; // S=4*qw
            q.w = 0.25f * S;
            q.x = (rot(2,1) - rot(1,2)) / S;
            q.y = (rot(0,2) - rot(2,0)) / S;
            q.z = (rot(1,0) - rot(0,1)) / S;
        } else if ((rot(0,0) > rot(1,1))&(rot(0,0) > rot(2,2))) {
            float S = std::sqrt(1.0f + rot(0,0) - rot(1,1) - rot(2,2)) * 2; // S=4*qx
            q.w = (rot(2,1) - rot(1,2)) / S;
            q.x = 0.25f * S;
            q.y = (rot(0,1) + rot(1,0)) / S;
            q.z = (rot(0,2) + rot(2,0)) / S;
        } else if (rot(1,1) > rot(2,2)) {
            float S = std::sqrt(1.0f + rot(1,1) - rot(0,0) - rot(2,2)) * 2; // S=4*qy
            q.w = (rot(0,2) - rot(2,0)) / S;
            q.x = (rot(0,1) + rot(1,0)) / S;
            q.y = 0.25f * S;
            q.z = (rot(1,2) + rot(2,1)) / S;
        } else {
            float S = std::sqrt(1.0f + rot(2,2) - rot(0,0) - rot(1,1)) * 2; // S=4*qz
            q.w = (rot(1,0) - rot(0,1)) / S;
            q.x = (rot(0,2) + rot(2,0)) / S;
            q.y = (rot(1,2) + rot(2,1)) / S;
            q.z = 0.25f * S;
        }
    }

    /**
     * @brief transform array of Quaternion to array of Euler Angle
     * @param quater array of Quaternion [x, y, z, w]
     * @param euler pointer pointing to an array of Euler Angle [theta_x, theta_y, theta_z]
     * @details Constraint: Cartesion Coordinate System \n
     *          Refrence: http://www.cnblogs.com/wqj1212/archive/2010/11/21/1883033.html
     */
    void Tools::Quaternion2Euler(const float *quater, float **euler)
    {
        float x = quater[0];
        float y = quater[1];
        float z = quater[2];
        float w = quater[3];
        (*euler)[0] = std::atan2 ( 2*(w*x+y*z), 1-2*(x*x+y*y) );
        (*euler)[1] = std::asin  ( 2*(w*y-z*x) );
        (*euler)[2] = std::atan2 ( 2*(w*z+x*y), 1-2*(y*y+z*z) );
    }

    /**
     * @brief transform array of Euler Angle to array of Quaternion
     * @param euler array of Euler Angle [theta_x, theta_y, theta_z]
     * @param quater pointer pointing to an array of Quaternion [x, y, z, w]
     */
    void Tools::Euler2Quaternion(const float *euler, float **quater)
    {
        // Abbreviations for the various angular functions
        float cy = std::cos(euler[2] * 0.5f);
        float sy = std::sin(euler[2] * 0.5f);
        float cr = std::cos(euler[0] * 0.5f);
        float sr = std::sin(euler[0] * 0.5f);
        float cp = std::cos(euler[1] * 0.5f);
        float sp = std::sin(euler[1] * 0.5f);

        (*quater)[3] = cy * cr * cp + sy * sr * sp;
        (*quater)[0] = cy * sr * cp - sy * cr * sp;
        (*quater)[1] = cy * cr * sp + sy * sr * cp;
        (*quater)[2] = sy * cr * cp - cy * sr * sp;
    }

    /**
     * @brief transform array of Rotation Matrix to array of Euler Angle
     * @param rm array of 3x3 Rotation Matrix
     * @param euler pointer pointing to an array of Euler Angle [theta_x, theta_y, theta_z]
     * @details R = Rz(φ)Ry(θ)Rx(ψ) \n
     *          Reference: https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-roational-matrix
     */
    void Tools::RotationMatrix2Euler(const float *rm, float **euler)
    {
        float rm11 = rm[0];
        float rm21 = rm[3];
        float rm31 = rm[6];
        float rm32 = rm[7];
        float rm33 = rm[8];
        (*euler)[0] = std::atan2( rm32 , rm33 );
        (*euler)[1] = std::atan2(-rm31 , std::sqrt(rm32*rm32+rm33*rm33) );
        (*euler)[2] = std::atan2( rm21 , rm11 );

        (*euler)[0] = (*euler)[0] ;//* 180.0 / 3.1415926;
        (*euler)[1] = (*euler)[1] ;//* 180.0 / 3.1415926;
        (*euler)[2] = (*euler)[2] ;//* 180.0 / 3.1415926;
    }

    /**
     * @brief Get the center of a mass of 3d points
     * @param vvPts a mass of 3d points
     * @param center the center to return
     */
    void Tools::GetCenterOfMass3D(const std::vector<Vector<3> > &vvPts, Vector<3> &center)
    {
        auto nSize = static_cast<unsigned int>(vvPts.size());
        if(nSize < 1)
            return;
        for(unsigned int i=0; i<nSize; i++)
        {
            center += vvPts[i];
        }
        center /= nSize;
    }

    /**
     * @brief estimate pose from some pairs of 3d points
     * @param ptsA
     * @param ptsB
     * @param se3AfromB
     * @details ICP using Sigular Values Decomposition
     */
    void Tools::ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB)
    {
        auto nSizeA = static_cast<unsigned int>(ptsA.size());
        auto nSizeB = static_cast<unsigned int>(ptsB.size());

        if(nSizeA==0 || nSizeA != nSizeB)
            return;

        // center of mass
        Vector<3> ptCenterA = TooN::Zeros;
        Vector<3> ptCenterB = TooN::Zeros;
        GetCenterOfMass3D(ptsA,ptCenterA);
        GetCenterOfMass3D(ptsB,ptCenterB);

        //compute W
        Matrix<3,3,float> W;
        for(unsigned int i=0; i<nSizeA; i++)
        {
            W += (ptsA[i]-ptCenterA).as_col() * (ptsB[i]-ptCenterB).as_row();
        }

        float determinantW = W(0,0)*W(1,1)*W(2,2) + W(0,1)*W(1,2)*W(2,0) + W(0,2)*W(1,0)*W(2,1) -
                           (W(0,0)*W(1,2)*W(2,1) + W(0,1)*W(1,0)*W(2,2) + W(0,2)*W(1,1)*W(2,0));
        assert(determinantW<1e-8);

        //svd decomposition
        SVD<3,3> svd(W);
        SO3<> so3 = svd.get_U()*svd.get_VT();
        Vector<3> translation = ptCenterA-so3*ptCenterB;

        //compose SE3
        se3AfromB.get_rotation() = so3;
        se3AfromB.get_translation() = translation;
    }

    /**
     * @brief estimate pose from some pairs of 3d points
     * @param ptsA
     * @param ptsB
     * @param se3AfromB
     * @details ICP using Quarternion Decomposition
     */
    void Tools::ICP_QD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB)
    {
        auto nSizeA = static_cast<unsigned int>(ptsA.size());
        auto nSizeB = static_cast<unsigned int>(ptsB.size());

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
        float eigenvalueMax = static_cast<float>(eigM.get_evalues()[3]);
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

