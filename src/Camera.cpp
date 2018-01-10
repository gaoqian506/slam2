

#include "Camera.h"

#include "MatrixToolbox.h"
#include <assert.h>
#include <stdlib.h>

namespace ww {

Camera::Camera() : radius(20), movement(0) {

	pos = Config::default_camera_pos;
	rotation[0] = 1;
	rotation[4] = 1;
	rotation[8] = 1;

	plane_n[2] = -1;
	plane_d = 0.1;

	plane[0] = Config::default_plane[0];
	plane[1] = Config::default_plane[1];
	plane[2] = Config::default_plane[2];

	original = NULL;
	gray = NULL;
	gradient[0] = NULL;
	gradient[1] = NULL;
	depth = NULL;
	points = NULL;
	depth_weight = NULL;

	mask = NULL;
	residual = NULL;
	warp_gradient = NULL;
	warp = NULL;
	of_weight = NULL;
	epi_weight = NULL;
	optical_flow = NULL;
	eof = NULL;
	//m_grad_residual = NULL;
	dut = NULL;
	//residual = NULL;

	int gc = Config::depth_grid_size_lsd6[0]*
		Config::depth_grid_size_lsd6[1];
	//depth_grid = (double*)malloc(grid_count*sizeof(double));
	depth_grid = (double*)malloc(gc*sizeof(double));
	memset(depth_grid, 0, gc*sizeof(double));
	//for (int i = 0; i < grid_count; i++) {
	//	depth_grid[i] = Config::default_depth_lsd6;
	//}
}

Camera::~Camera() {

	free(depth_grid);
}


Vec3f Camera::unproject(const float& u, const float& v, const float& d) {
	assert(0);
	return Vec3f();
}

Vec3f Camera::project(const Vec3f& pt) {
	assert(0);
	return Vec3f();

}

void Camera::rotation_warp(Image*& out, bool inverse/* = true*/) {

	if (!gray) { return; }
	double K[9] = {
		intrinsic.f, 0, intrinsic.cx,
		0, intrinsic.f, intrinsic.cy,
		0, 0, 1
	};
	double f1 = 1.0 / intrinsic.f;
	double iK[9] = {
		f1, 0, -intrinsic.cx*f1,
		0, f1, -intrinsic.cy*f1,
		0, 0, 1
	};
	Vec9d R = rotation;
	if (inverse) {
		R = MatrixToolbox::transpose_3x3(rotation);
	}
	Vec9d H = MatrixToolbox::mult_matrix_3x3(R, iK);
	H = MatrixToolbox::mult_matrix_3x3(K, H);
	gray->warp(H, out);

}

Vec2d Camera::image_epi_point() {

	// double z1 = 1.0 / (epi_point[2] + 0.0001);
	// return Vec2d(
	// 	intrinsic.f*epi_point[0]*z1+intrinsic.cx,
	// 	intrinsic.f*epi_point[1]*z1+intrinsic.cy
	// );

	double z1 = 1.0 / (pos[2] + 0.0001);
	return Vec2d(
		intrinsic.f*pos[0]*z1+intrinsic.cx,
		intrinsic.f*pos[1]*z1+intrinsic.cy
	);	

}

} // namespace
