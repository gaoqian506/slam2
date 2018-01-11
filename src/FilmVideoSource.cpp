

#include "FilmVideoSource.h"
#include "CvImage.h"
#include "Config.h"

namespace ww {


FilmVideoSource::FilmVideoSource(const char* path) {

	m_capture.open(path);
	assert(m_capture.isOpened());

	for (int i = 0; i < Config::num_skip_frames; i++) {
		m_capture.grab();
	}
	//m_source->skip(Config::num_skip_frames);
	
	//cv::namedWindow("video", CV_WINDOW_NORMAL);

}


bool FilmVideoSource::read(Image*& image) {

	if (image == NULL) { 
		image = new CvImage(); 
	}

	CvImage* cv_image = static_cast<CvImage*>(image);
	
	assert(cv_image);
	
	//m_capture.read(cv_image->cv_mat());
	if (m_capture.read(cv_image->cv_mat())) {
		//cv::imshow("video", cv_image->cv_mat());  
		//cv::waitKey(10);
		return true;	
	}
	else {
		return false;
	}


}


} // namespace

