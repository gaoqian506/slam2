


#ifndef __WW_MATRIX_TOOLBOX_HEADER__
#define __WW_MATRIX_TOOLBOX_HEADER__

#include "Vectors.h"


namespace ww {



class MatrixToolbox {

public:

static Vec9d inv_matrix_3x3(Vec9d from);
static void update_rotation(Vec9d& R, const Vec3d& eula);
static void rectify_rotation(Vec9d& R);

static void identity(Vec16d& M);

static Vec9d mult_matrix_3x3(const Vec9d& m1, const Vec9d& m2);

static Vec3d min_eigen_vector_3x3(const Vec9d& A);

static Vec9d transpose_3x3(const Vec9d& R);
};

}// namespace

#endif
