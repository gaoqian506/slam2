
#ifndef __WW_CAMERA_HEADER__
#define __WW_CAMERA_HEADER__


#include "Image.h"
#include "Vectors.h"
#include "Config.h"

namespace ww {


struct Intrinsic {
	Intrinsic() : f(500), cx(320), cy(240) {

	}
	double f;
	double cx;
	double cy;
};

struct CanonicalIntrinsic {
	// Intrinsic() : fx(500), cx(320), cy(240) {

	// }
	double fx;
	double fy;
	double cx;
	double cy;
};

class Camera {

public:
	Camera();
	~Camera();
	
	Vec3f unproject(const float& u, const float& v, const float& d);
	Vec3f project(const Vec3f& pt);
	void rotation_warp(Image*& out, bool inverse = true);
	Vec2d image_epi_point();


	Vec3d epi_point;
	Vec3d pos;
	Vec3d alpha;
	Vec9d rotation;
	Vec3d plane_n;
	double plane_d;
	double plane[3];	// theta, phi, 1.0/rou
	double movement;
	double radius;
	Intrinsic intrinsic;
	CanonicalIntrinsic canonical_intrinsic;

	
	Image* original;
	Image* gray;
	Image* gradient[2];
	Image* depth;
	Image* depth2;
	Image* points;
	Image* epi_line;
	Image* depth_weight;
	Image* ddepth_weight;

	Image* mask;
	Image* mask2;
	Image* residual;
	Image* residual2;
	Image* warp_gradient;
	Image* warp_gradient2;
	Image* warp;
	Image* warp2;
	Image* of_weight;
	Image* epi_weight;
	Image* optical_flow;
	//Image* m_grad_residual;
	Image* dut;	
	Image* eof;
	Image* ddepth;
	Image* of_residual;
	Image* plane_pi;

//Config::depth_grid_size_lsd6[0],
//		Config::depth_grid_size_lsd6[1]
//	Config::depth_grid_size_lsd6[1]
	double* depth_grid;
};


}

#endif
