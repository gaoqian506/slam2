

#ifndef __WW_VIDEO_SOURCE_HEADER__
#define __WW_VIDEO_SOURCE_HEADER__

#include "Image.h"

namespace ww {



class VideoSource {


public:
	virtual bool read(Image*& image) = 0;

};


}

#endif
