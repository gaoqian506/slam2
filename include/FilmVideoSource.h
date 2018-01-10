
#ifndef __WW_FILM_VIDEO_SOURCE_HEADER__
#define __WW_FILM_VIDEO_SOURCE_HEADER__


#include "VideoSource.h"
#include "opencv2/highgui/highgui.hpp"


namespace ww {



class FilmVideoSource : public VideoSource {

public:
	FilmVideoSource(const char* path);

	virtual bool read(Image*& image);

private:
	cv::VideoCapture m_capture;
};


}

#endif
