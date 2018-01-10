
#ifndef __WW_IMAGE_HEADER__
#define __WW_IMAGE_HEADER__

#include "Vectors.h"

namespace ww {



class Image {

public:

	enum DataType { UByte, Byte, UShort, Short, Int, Float32, Float64 };

	virtual void* data() { return 0; }
	virtual int width() { return 0; }
	virtual int height() { return 0; }
	
	virtual void gray(Image*& out) { }
	virtual void sobel_x(Image*& out) { }
	virtual void sobel_y(Image*& out) { }
	virtual void subtract(Image* b, Image*& out) { }
	virtual void copy_to(Image*& out) { }
	virtual void convert_to(Image*& out, DataType type, double alpha = 1.0, double beta = 0.0) { }
	
	virtual void set(double v) { }
	virtual void save(const char* path) { }
	virtual void resize(Image*& out) { }
	virtual double average2() { return 0; }
	virtual double abs_mean() { return 0; }
	virtual DataType type() { return UByte; }
	virtual int channels() { return 0; }
	virtual bool empty() { return true; }
	virtual void min_max(double* min, double* max) {}
	virtual void* at(int idx) { return 0; }
	virtual void merge(Image* other, Image*& out) {}
	virtual void add(Image* right) {}
	virtual void warp(Vec9d H, Image*& out) {}
	virtual void random(double low, double high) {}
	virtual int count_nozero() {}

	//virtual float sample(const float& a, const float& b) { return 0; }
	

};


}

#endif
