
#ifndef __WW_CV_IMAGE_HEADER__
#define __WW_CV_IMAGE_HEADER__

#include "Image.h"
#include "opencv2/core/core.hpp"


namespace ww {



class CvImage : public Image {

public:

	CvImage();
	CvImage(const int& width, const int& height, const DataType& type, const int& channels = 1);

	cv::Mat& cv_mat() { return m_cv_mat; }
	const cv::Mat& cv_Mat() const { return m_cv_mat; }
	
	virtual void* data();
	virtual int width();
	virtual int height();
	
	virtual void gray(Image*& out);
	virtual void sobel_x(Image*& out);
	virtual void sobel_y(Image*& out);
	virtual void subtract(Image* b, Image*& out);
	virtual void copy_to(Image*& out);
	virtual void convert_to(Image*& out, DataType type, double alpha = 1.0, double beta = 0.0);
	
	virtual void set(double v);
	virtual void save(const char* path);
	//virtual float sample(const float& a, const float& b) { return 0; }
	virtual void resize(Image*& out);
	virtual double average2();
	virtual double abs_mean();
	virtual DataType type();
	virtual int channels();
	virtual bool empty();
	virtual void min_max(double* min, double* max);
	virtual void* at(int idx);
	virtual void merge(Image* other, Image*& out);
	virtual void add(Image* right);
	virtual void warp(Vec9d H, Image*& out);
	virtual void random(double low, double high);
	virtual int count_nozero();

private:

	cv::Mat m_cv_mat;

};


}

#endif
