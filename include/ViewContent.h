
#ifndef __WW_VIEW_CONTENT_HEADER__
#define __WW_VIEW_CONTENT_HEADER__

#include "Camera.h"
#include <iostream>

namespace ww {

class ViewContent;
class DisplayDelegate;


class DisplayDelegate {

public:
	virtual void display_with(ViewContent* cv) {
		std::cout << "DisplayDelegate::display_with" << std::endl;
	}
};



class ViewContent {

public:

	enum BuildFlag {
		BuildReadFrame = 1,
		BuildOpticalFlow = 2,
		BuildEpipolar = 4,
		BuildTranslate = 8,
		BuildDepth = 16,
		BuildKeyframe = 32,
		BuildIterate = 64,
		BuildSequence = 128,
		BuildAll = 256-1,
	};

	ViewContent() : m_display_delegate(0) {
		std::cout << "ViewContent::ViewContent" << std::endl;
	}

	virtual void start() = 0;
	virtual void stop() = 0;
	virtual Camera** get_cameras() = 0;
	virtual int get_camera_count() = 0;
	virtual Camera* get_current_frame() = 0;
	virtual bool changed() = 0;
	virtual Image* get_debug_image(int iid, int kid, Image** weight = 0) = 0;
	virtual void push_manauly() = 0;
	virtual void func_manualy(int idx) = 0;
	virtual char* pixel_info(const Vec2d& u, int kid) = 0;
	virtual Image* get_optical_flow() = 0;

	virtual void build(BuildFlag flag = BuildAll) = 0;


	void set_display_delegate(DisplayDelegate* dd) {
		std::cout << "ViewContent::set_display_delegate" << std::endl;
		m_display_delegate = dd;
	}
	
protected:
	DisplayDelegate* m_display_delegate;
	

};


} // namespace

#endif


/*

	virtual void tick() = 0;

*/
