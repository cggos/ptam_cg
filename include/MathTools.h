#ifndef __MathTools__H
#define __MathTools__H

#include <vector>
#include <cmath>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

using namespace TooN;

namespace cg
{

class MathTool
{
public:
	static void LHFromRH(SE3<> rhm,SE3<> &lhm);
	static float* se3_to_float(SE3<> tform);
	static void se3_to_float(SE3<> tform, float* mat);
	static void se3_to_euler(SE3<> tform, float* mat);

	static void SE3_to_RM(SE3<> se3, float* rm);
	static void Quaternion2Euler(float *quater, float **euler);
	static void Euler2Quaternion(float *euler, float **quater);
	static void RotationMatrix2Euler(float *rm, float **euler);
	static void RotationMatrixEnhanced2Euler(float *rme, float **euler);
	
	static void ICP_SVD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
	static void ICP_QD(const std::vector<Vector<3> > &ptsA, const std::vector<Vector<3> > &ptsB, SE3<> &se3AfromB);
};

}

#endif
