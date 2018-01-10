
#include "MatrixToolbox.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"


namespace ww {


Vec9d MatrixToolbox::inv_matrix_3x3(Vec9d from) {
/*
	from = 0;
	from[0] = 500;
	from[2] = 320;
	from[4] = 500;
	from[5] = 240;
	from[8] = 1;
*/

	Vec9d to;
	cv::Mat A = cv::Mat(3, 3, CV_64F, from.ptr());
	cv::Mat invA = cv::Mat(3, 3, CV_64F, to.ptr());
	cv::invert(A, invA);
	
	//Vec9d test;
	//cv::Mat T = A * invA;
	//memcpy(test.ptr(), T.data, sizeof(test));
	//cv::Mat T2 = A.inv();
	return to;
}

void MatrixToolbox::update_rotation(Vec9d& R, const Vec3d& eula) {

	//double dR[9] = {
	//	1, -eula[2], eula[1], 
	//	eula[2], 1, -eula[0],
	//	-eula[1], eula[0], 1
	//};

	//[ cos(b)*cos(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c), sin(a)*sin(c) + cos(a)*cos(c)*sin(b)]
//[ cos(b)*sin(c), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(a)*sin(b)*sin(c) - cos(c)*sin(a)]
//[       -sin(b),                        cos(b)*sin(a),                        cos(a)*cos(b)]

	double a = eula[0];
	double b = eula[1];
	double c = eula[2];
	double dR[9] = {
		cos(b)*cos(c), 
		cos(c)*sin(a)*sin(b) - cos(a)*sin(c), 
		sin(a)*sin(c) + cos(a)*cos(c)*sin(b),
		cos(b)*sin(c),
		cos(a)*cos(c) + sin(a)*sin(b)*sin(c), 
		cos(a)*sin(b)*sin(c) - cos(c)*sin(a),
		-sin(b),
		cos(b)*sin(a),
		cos(a)*cos(b)
	};


	cv::Mat cvR = cv::Mat(3, 3, CV_64F, R.val);
	cv::Mat cvdR = cv::Mat(3, 3, CV_64F, dR);
	cvR = cvdR  * cvR;
	
	cv::SVD svd(cvR);
	cvR = svd.u*svd.vt;
}
void MatrixToolbox::rectify_rotation(Vec9d& R) {

	assert(0);
}

void MatrixToolbox::identity(Vec16d& M) {

	memset(M.val, 0, sizeof(M.val));
	M[0] = 1;
	M[5] = 1;
	M[10] = 1;
	M[15] = 1;

}

Vec9d MatrixToolbox::mult_matrix_3x3(const Vec9d& m1, const Vec9d& m2) {
	
	Vec9d re;
	re[0] = m1[0]*m2[0]+m1[1]*m2[3]+m1[2]*m2[6];
	re[1] = m1[0]*m2[1]+m1[1]*m2[4]+m1[2]*m2[7];
	re[2] = m1[0]*m2[2]+m1[1]*m2[5]+m1[2]*m2[8];

	re[3] = m1[3]*m2[0]+m1[4]*m2[3]+m1[5]*m2[6];
	re[4] = m1[3]*m2[1]+m1[4]*m2[4]+m1[5]*m2[7];
	re[5] = m1[3]*m2[2]+m1[4]*m2[5]+m1[5]*m2[8];

	re[6] = m1[6]*m2[0]+m1[7]*m2[3]+m1[8]*m2[6];
	re[7] = m1[6]*m2[1]+m1[7]*m2[4]+m1[8]*m2[7];
	re[8] = m1[6]*m2[2]+m1[7]*m2[5]+m1[8]*m2[8];

	return re;
}

Vec3d MatrixToolbox::min_eigen_vector_3x3(const Vec9d& A) {

	double A_[9];
	memcpy(A_, A.val, sizeof(A_));

	cv::Mat M = cv::Mat(3, 3, CV_64F, A_);
	cv::SVD svd(M);
	return Vec3d(svd.vt.ptr<double>(2));

}

Vec9d MatrixToolbox::transpose_3x3(const Vec9d& M) {
	Vec9d T;
	T[0] = M[0];
	T[1] = M[3];
	T[2] = M[6];

	T[3] = M[1];
	T[4] = M[4];
	T[5] = M[7];

	T[6] = M[2];
	T[7] = M[5];
	T[8] = M[8];

	return T;
}

} // namespace


/*******************************


	//svd.u.col(2).copyTo(t);
	//int a = svd.vt.type();
	//int b = CV_64F;
	//return Vec3d();

cos(eula[1])*cos(eula[0])-cos(eula[2])*sin(eula[1])*sin(eula[0]), 
		-cos(eula[2])*cos(eula[0])*sin(eula[1])-cos(eula[1])*sin(eula[0]),
		sin(eula[1])*sin(eula[2]),
		cos(eula[0])*sin(eula[1])+cos(eula[1])*cos(eula[2])*sin(eula[0]),
		cos(eula[1])*cos(eula[2])*cos(eula[0])-sin(eula[1])*sin(eula[0]), 
		-cos(eula[1])*sin(eula[2]),
		sin(eula[2])*sin(eula[0]),
		cos(eula[0])*sin(eula[2]),
		cos(eula[2])
	
	//A.inverse(invA);
	
	
*********************************/
