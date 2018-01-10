
#ifndef __WW_RECTANGLE_HEADER__
#define __WW_RECTANGLE_HEADER__



namespace ww {

struct Rectangle {

	Rectangle(const double& l, const double& b, const double& w, const double& h) : left(l), bottom(b), width(w), height(h) {
	
	}

	double left;
	double bottom;
	double width;
	double height;

};


}

#endif
