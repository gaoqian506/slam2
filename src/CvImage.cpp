
#include "CvImage.h"
#include "Config.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace ww {

	CvImage::CvImage() {
		
	}
	
	CvImage::CvImage(const int& width, const int& height, const DataType& type, const int& channels) {
		switch(type) {
		case Float32:
			m_cv_mat = cv::Mat(height, width, CV_32FC(channels));
			break;
		case UByte:
			m_cv_mat = cv::Mat(height, width, CV_8UC(channels));
			break;
		}
		m_cv_mat.setTo(0);
	}

void* CvImage::data() {

	return m_cv_mat.data;

}
int CvImage::width() {

	return m_cv_mat.cols;

}
int CvImage::height() {
	return m_cv_mat.rows;
}

void CvImage::gray(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::Mat gray;
	cv::cvtColor(m_cv_mat, gray/*cv_out->cv_mat()*/, CV_BGR2GRAY); 
	
	
	if (Config::smooth_input_image) {
		cv::Mat grayf;
		gray.convertTo(grayf, CV_32F, 1.0/255.0);
		cv::GaussianBlur(grayf, cv_out->m_cv_mat,  cv::Size( 3, 3 ), 0, 0 );
	}
	else {
		gray.convertTo(cv_out->m_cv_mat, CV_32F, 1.0/255.0);
	}
	
}
void CvImage::sobel_x(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::Sobel(m_cv_mat, cv_out->cv_mat(), m_cv_mat.depth(), 1, 0, 3, 0.25);
}
void CvImage::sobel_y(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::Sobel(m_cv_mat, cv_out->cv_mat(), m_cv_mat.depth(), 0, 1, 3, 0.25);

}
void CvImage::subtract(Image* b, Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	CvImage* cv_b = static_cast<CvImage*>(b);
	cv::subtract(m_cv_mat, cv_b->cv_mat(), cv_out->cv_mat());
	//int t = cv_out->cv_mat().type();
	//cv_b = 0;
}
void CvImage::copy_to(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	m_cv_mat.copyTo(cv_out->cv_mat());

}
void CvImage::convert_to(Image*& out, DataType type, double alpha/* = 1.0*/, double beta/* = 0.0*/) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	
	int t = CV_32F;
	switch (type) {
	
	case Float32:
		t = CV_32F;
		break;
	}
	m_cv_mat.convertTo(cv_out->cv_mat(), t, alpha, beta);
}

void CvImage::set(double v) {

	m_cv_mat.setTo(v);
}

void CvImage::save(const char* path) {

	double minv = 0.0, maxv = 0.0;  
	double* minp = &minv;  
	double* maxp = &maxv;  
	minMaxIdx(m_cv_mat, &minv, &maxv); 
	cv::Mat stand = (m_cv_mat-minv) / (maxv-minv) * 255;
	cv::imwrite(path, stand);
}

void CvImage::resize(Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	
	int w = m_cv_mat.cols;
	int h = m_cv_mat.rows;
	
	if (w > Config::max_width) {
	
		int th = (int)(((double)Config::max_width)/w*h);
	
	cv::resize(m_cv_mat, cv_out->cv_mat(), cv::Size(Config::max_width, th), 0, 0, CV_INTER_CUBIC);
	}
	else {
		copy_to(out);
	}
}

double CvImage::average2() {

	assert(0);
	cv::Mat abs = cv::abs(m_cv_mat);
	return cv::mean(abs)[0]; 
}

double CvImage::abs_mean() {

	cv::Mat abs = cv::abs(m_cv_mat);
	return cv::mean(abs)[0]; 
}

Image::DataType CvImage::type() {

	return (DataType)(m_cv_mat.type() % 8);

}
int CvImage::channels() {
	return m_cv_mat.channels();
}

bool CvImage::empty() {

	return m_cv_mat.empty();

}

 void CvImage::min_max(double* min, double* max) {

	cv::minMaxIdx(m_cv_mat, min, max);
}

void* CvImage::at(int idx) {
	return m_cv_mat.data + idx * m_cv_mat.step.buf[1];
	//assert(0);
}

void CvImage::merge(Image* other, Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_other = static_cast<CvImage*>(other);
	CvImage* cv_out = static_cast<CvImage*>(out);
	const cv::Mat ml[2] = { m_cv_mat, cv_other->m_cv_mat };
	cv::merge(ml, 2, cv_out->m_cv_mat);

}

void CvImage::add(Image* right) {

	assert(right);
	CvImage* cv_right = static_cast<CvImage*>(right);
	m_cv_mat += cv_right->m_cv_mat;

}

void CvImage::warp(Vec9d H, Image*& out) {

	if (out == NULL) {
		out = new CvImage();
	}
	CvImage* cv_out = static_cast<CvImage*>(out);
	cv::Mat M = cv::Mat(3, 3, CV_64F, H.val);
	cv::warpPerspective(m_cv_mat, cv_out->m_cv_mat, M, m_cv_mat.size());

}

void CvImage::random(double low, double high) {

	cv::randu(m_cv_mat, low, high);
}

int CvImage::count_nozero() {
	
	return cv::countNonZero(m_cv_mat);
}
//float CvImage::sample(const float& a, const float& b) {

	//assert(m_cv_mat.type() == CV_32F);
	
	//return 0; 
 
 //}


} // namespace
