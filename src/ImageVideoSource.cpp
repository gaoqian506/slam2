
#include "ImageVideoSource.h"
#include "CvImage.h"

namespace ww {


ImageVideoSource::ImageVideoSource(char** names, const int count) : m_looply(true) {

	for (int i = 0; i < count; i++) {
		m_images.push_back(cv::imread(names[i]));
	}
	m_cursor = m_images.begin();
	
}

bool ImageVideoSource::read(Image*& image) {

	if (m_cursor == m_images.end()) {
	
		if (m_images.empty() || !m_looply) {
			return false;
		}
		else {
			m_cursor = m_images.begin();
		}
	}
	
	if (image == NULL) { 
		image = new CvImage(); 
	}

	CvImage* cv_image = static_cast<CvImage*>(image);
	m_cursor->copyTo(cv_image->cv_mat());
	m_cursor++;
	
	return true;
}

}

