

#include "FilmVideoSource.h"
#include "ImageVideoSource.h"
#include "Slam.h"
#include "View.h"
#include <GL/glut.h>




int main(int argc, char** argv) {

	glutInit(&argc, argv);
	
	ww::VideoSource* vs = NULL;
	if (argc < 2) {
		std::cout << "please set video name." << std::endl;
		return 0;
	}
	else if (argc == 2) {
		vs = new ww::FilmVideoSource(argv[1]);	
	}
	else {
		vs = new ww::ImageVideoSource(argv+1, argc-1);
	}

	ww::ViewContent* vc = new ww::Slam(vs);
	ww::View view(vc);
	view.run();
	
	delete vs;
	delete vc;
	

	return 0;

}
