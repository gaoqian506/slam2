
#include "SpaceToolbox.h"
#include "MatrixToolbox.h"
#include <assert.h>
#include <math.h>

namespace ww {


std::vector<Vec3d> SpaceToolbox::unproject(const std::vector<Vec3d>& from) {

	assert(0);
	return std::vector<Vec3d>();
	
}

std::vector<Vec3d> SpaceToolbox::unproject(const Intrinsic& intrinsic, const std::vector<Vec3d>& from) {

	std::vector<Vec3d> to;
	double invf = 1/intrinsic.f;
	//double invk[3] = { 1/intrinsic.f, intrinsic.cx/intrinsic.f, intrinsic.cy/intrinsic.f };
	
	for (std::vector<Vec3d>::const_iterator itr = from.begin(); itr != from.end(); itr++) {
		const double* uvz = itr->val;
		to.push_back(Vec3d(
			uvz[2]*invf*(uvz[0]-intrinsic.cx),
			uvz[2]*invf*(uvz[1]-intrinsic.cy),
			uvz[2]));
	}

	return to;
	
}

void SpaceToolbox::init_intrinsic(Intrinsic& intri, double fovy, int width, int height) {

	double theta = fovy / 180 * M_PI;
	intri.f = height * 0.5 / tan(0.5*theta);
	intri.cx = width * 0.5;
	intri.cy = height * 0.5;
}

void SpaceToolbox::init_canonical_intrinsic(CanonicalIntrinsic& intri, double fovy, int width, int height) {

	double aspect = (double)width / height;
	double theta = fovy / 180 * M_PI;
	intri.fy = 0.5 / tan(0.5*theta);
	intri.fx = intri.fy / aspect;
	intri.cx = 0.5;
	intri.cy = 0.5;
}

Vec9d SpaceToolbox::make_rotation(const double& thex, const double& they) {

	Vec9d R;
	R[6] = cos(thex)*sin(they);
	R[7] = -sin(thex);
	R[8] = cos(thex)*cos(they);

	double norm = sqrt(R[6]*R[6]+R[8]*R[8]);
	R[0] = R[8] / norm;
	R[1] = 0;
	R[2] = -R[6] / norm;

	R[3] = R[7]*R[2]-R[8]*R[1];
	R[4] = R[8]*R[0]-R[6]*R[2];
	R[5] = R[6]*R[1]-R[7]*R[0];

	return R;

}

void SpaceToolbox::rotate(Vec16d& T, const Vec9d& R) {
	Vec16d A(T);
	
	T[0] = R[0]*A[0]+R[1]*A[4]+R[2]*A[8];
	T[1] = R[0]*A[1]+R[1]*A[5]+R[2]*A[9];
	T[2] = R[0]*A[2]+R[1]*A[6]+R[2]*A[10];
	T[3] = R[0]*A[3]+R[1]*A[7]+R[2]*A[11];

	T[4] = R[3]*A[0]+R[4]*A[4]+R[5]*A[8];
	T[5] = R[3]*A[1]+R[4]*A[5]+R[5]*A[9];
	T[6] = R[3]*A[2]+R[4]*A[6]+R[5]*A[10];
	T[7] = R[3]*A[4]+R[4]*A[7]+R[5]*A[11];

	T[8] = R[6]*A[0]+R[7]*A[4]+R[8]*A[8];
	T[9] = R[6]*A[1]+R[7]*A[5]+R[8]*A[9];
	T[10] = R[6]*A[2]+R[7]*A[6]+R[8]*A[10];
	T[11] = R[6]*A[3]+R[7]*A[7]+R[8]*A[11];
	
}

void SpaceToolbox::translate(Vec16d& T, const Vec3d& t) {

	T[3] += t[0];
	T[7] += t[1];
	T[11] += t[2];
	
}

void SpaceToolbox::make_KRKi(Intrinsic intri0, const Vec9d& R, Intrinsic intri1, double* out, bool invR/* = false*/) {

	Vec9d Ki;
	double invf = 1.0 / intri0.f;
	Ki[0] = invf;
	Ki[2] = -intri0.cx * invf;
	Ki[4] = invf;
	Ki[5] = -intri0.cy * invf;
	Ki[8] = 1;

	Vec9d RR = R;
	if (invR) { RR = MatrixToolbox::transpose_3x3(R); }
	Vec9d RKi = MatrixToolbox::mult_matrix_3x3(RR, Ki);

	Vec9d K;
	K[0] = intri1.f;
	K[2] = intri1.cx;
	K[4] = intri1.f;
	K[5] = intri1.cy;
	K[8] = 1;

	Vec9d KRKi = MatrixToolbox::mult_matrix_3x3(K, RKi);

	memcpy(out, KRKi.val, sizeof(KRKi));

}

} // namespace

