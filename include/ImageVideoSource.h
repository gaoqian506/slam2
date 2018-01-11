
#ifndef __WW_IMAGE_VIDEO_SOURCE_HEADER__
#define __WW_IMAGE_VIDEO_SOURCE_HEADER__


#include "VideoSource.h"
#include "opencv2/highgui/highgui.hpp"


namespace ww {



class ImageVideoSource : public VideoSource {

public:
	ImageVideoSource(char** names, const int count);
	ImageVideoSource(std::vector<std::string> names);

	virtual bool read(Image*& image);
	virtual void skip(int num_frames) {}

private:
	std::vector<cv::Mat> m_images;
	std::vector<cv::Mat>::iterator m_cursor;
	bool m_looply;
};


}

#endif
