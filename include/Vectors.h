#ifndef __WW_VECTORS_HEADER__
#define __WW_VECTORS_HEADER__

#include <memory.h>
#include <math.h>

namespace ww {

struct Vec2f {

	Vec2f(const float& a = 0, const float& b = 0) {
		val[0] = a;
		val[1] = b;
	}
	
	float& operator[](int idx) {
		return val[idx];
	}


	Vec2f& operator+=(const Vec2f& right) {
	
		val[0] += right.val[0];
		val[1] += right.val[1];
		return *this;
	}

	Vec2f& operator-=(const Vec2f& right) {
	
		val[0] -= right.val[0];
		val[1] -= right.val[1];
		return *this;
	}

	Vec2f operator-(const Vec2f& right) {
		Vec2f r(*this);
		r -= right;
		return r;
	}


	Vec2f& operator*=(float f) {
	
		val[0] *= f;
		val[1] *= f;
		return *this;
	}

	Vec2f& operator/=(float f) {
	
		val[0] /= f;
		val[1] /= f;
		return *this;
	}

	Vec2f operator*(float f) {
		Vec2f r(*this);
		r *= f;
		return r;
	}
	
	
	float length2() { return val[0]*val[0]+val[1]*val[1]; }	
	float val[2];
};

struct Vec3f {

	Vec3f(const float& a = 0, const float& b = 0, const float& c = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
	}
	
	float& operator[](int idx) {
		return val[idx];
	}

	const float& operator[](int idx) const {
		return val[idx];
	}

	Vec3f& operator/=(float d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		return *this;
	}

	Vec3f& operator*=(float d) {
	
		val[0] *= d;
		val[1] *= d;
		val[2] *= d;
		return *this;
	}	

	Vec3f& operator+=(const Vec3f& right) {
	
		val[0] += right[0];
		val[1] += right[1];
		val[2] += right[2];
		return *this;
	}	

	Vec3f operator*(float f) {
		Vec3f r(*this);
		r *= f;
		return r;
	}	

	Vec3f operator/(float f) {
		Vec3f r(*this);
		r /= f;
		return r;
	}	

	Vec3f operator+(const Vec3f& right) {
		Vec3f r(*this);
		r += right;
		return r;
	}

	
	float length2() { return val[0]*val[0]+val[1]*val[1]+val[2]*val[2]; }
	float val[3];
};

struct Vec4f {

	Vec4f(const float& a = 0, const float& b = 0, const float& c = 0, const float& d = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
		val[3] = d;
	}
	
	float& operator[](int idx) {
		return val[idx];
	}
	
	Vec4f& operator/=(float d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		val[3] /= d;
		return *this;
	}
	
	float val[4];
};


struct Vec2d {

	Vec2d(const double& a = 0, const double& b = 0) {
	
		val[0] = a;
		val[1] = b;
	}
	
	double& operator[](int idx) {
		return val[idx];
	}
	
	const double& operator[](int idx) const {
		return val[idx];
	}
	
	Vec2d& operator+=(const Vec2d& right) {
	
		val[0] += right[0];
		val[1] += right[0];
		return *this;
	}
	Vec2d& operator/=(double d) {
	
		val[0] /= d;
		val[1] /= d;
		return *this;
	}
	
	double length2() { return val[0]*val[0]+val[1]*val[1]; }
	double val[2];
};


struct Vec3d {

	Vec3d(const double& a = 0, const double& b = 0, const double& c = 0) {
	
		val[0] = a;
		val[1] = b;
		val[2] = c;
	}

	Vec3d(double* p) {
		memcpy(val, p, sizeof(val));
	}
	
	double& operator[](int idx) {
		return val[idx];
	}
	
	const double& operator[](int idx) const {
		return val[idx];
	}
	
	Vec3d& operator+=(const Vec3d& right) {
	
		val[0] += right[0];
		val[1] += right[1];
		val[2] += right[2];
		return *this;
	}
	Vec3d& operator/=(double d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		return *this;
	}

	Vec3d& operator*=(double d) {
	
		val[0] *= d;
		val[1] *= d;
		val[2] *= d;
		return *this;
	}

	Vec3d operator*(double d) {
		Vec3d r(*this);
		r *= d;
		return r;
	}

	double dot(const Vec3d& r) {
		return val[0]*r.val[0]+val[1]*r.val[1]+val[2]*r.val[2];
	}
	
	double length2() { 
		return val[0]*val[0]+val[1]*val[1]+val[2]*val[2]; 
	}

	double length() { 
		return sqrt(val[0]*val[0]+val[1]*val[1]+val[2]*val[2]); 
	}
	

	Vec3d& normalize() {
	
		double n = length();
		if (n > 0.0000001) {
			return *this /= n;
		}
		else {
			return *this;	
		}
		
	}

	double val[3];
};

struct Vec9d {

	Vec9d() {
		memset(val, 0, sizeof(val));
	}

	Vec9d(double* p) {
		memcpy(val, p, sizeof(val));
	}
	
	double& operator[](int idx) {
		return val[idx];
	}
	
	const double& operator[] (int idx) const {
		return val[idx];
	}
	
	Vec9d& operator/=(double d) {
	
		val[0] /= d;
		val[1] /= d;
		val[2] /= d;
		val[3] /= d;
		val[4] /= d;
		val[5] /= d;
		val[6] /= d;
		val[7] /= d;
		val[8] /= d;
		return *this;
	}
	
	Vec9d& operator=(const double& d) {
	
		val[0] = d;
		val[1] = d;
		val[2] = d;
		val[3] = d;
		val[4] = d;
		val[5] = d;
		val[6] = d;
		val[7] = d;
		val[8] = d;
		return *this;
	}
	
	double* ptr() { return val; }
	
	double val[9];
	
};

struct Vec16d {

	Vec16d() {
		memset(val, 0, sizeof(val));
	}
	
	Vec16d(const Vec16d& c) {
		memcpy(val, c.val, sizeof(val));
	}

	double& operator[](int idx) {
		return val[idx];
	}

	const double& operator[] (int idx) const {
		return val[idx];
	}

	double val[16];
};



}

#endif


/*



	void operator=(double v) {
	
		for (int i = 0; i < 9; i++) {
			val[i] = v;
		}
	}


*/
