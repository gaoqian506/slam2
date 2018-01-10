#ifndef __WW_SPACE_TOOLBOX_HEADER__
#define __WW_SPACE_TOOLBOX_HEADER__

#include "Vectors.h"
#include "Camera.h"
#include <vector>


namespace ww {



class SpaceToolbox {

public:

static std::vector<Vec3d> unproject(const std::vector<Vec3d>& from);

static std::vector<Vec3d> unproject(const Intrinsic& intrinsic, const std::vector<Vec3d>& from);

static void init_intrinsic(Intrinsic& intri, double fovy, int width, int height);
static void init_canonical_intrinsic(CanonicalIntrinsic& intri, double fovy, int width, int height);

static Vec9d make_rotation(const double& thex, const double& they);

static void rotate(Vec16d& T, const Vec9d& R);

static void translate(Vec16d& T, const Vec3d& t);

static void make_KRKi(Intrinsic intri0, const Vec9d& R, Intrinsic intri1, double* out, bool invR = false);

};


}

#endif
