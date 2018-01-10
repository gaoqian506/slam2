
#ifndef __WW_IMAGE_VIDEO_SOURCE_HEADER__
#define __WW_IMAGE_VIDEO_SOURCE_HEADER__


#include "VideoSource.h"
#include "opencv2/highgui/highgui.hpp"


namespace ww {



class ImageVideoSource : public VideoSource {

public:
	ImageVideoSource(char** names, const int count);

	virtual bool read(Image*& image);

private:
	std::vector<cv::Mat> m_images;
	std::vector<cv::Mat>::iterator m_cursor;
	bool m_looply;
};


}

#endif
