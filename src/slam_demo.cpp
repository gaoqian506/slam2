

#include "FilmVideoSource.h"
#include "ImageVideoSource.h"
#include "Slam.h"
#include "View.h"
#include <GL/glut.h>




int main(int argc, char** argv) {

	glutInit(&argc, argv);

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "-config") == 0 && i < argc - 1) {
			ww::Config::load(argv[i+1]);
		}
	}
	
	ww::VideoSource* vs = NULL;
	// if (argc < 2) {
	// 	std::cout << "please set video name." << std::endl;
	// 	return 0;
	// }
	// else if (argc == 2) {
	// 	vs = new ww::FilmVideoSource(argv[1]);	
	// }
	// else {
	// 	vs = new ww::ImageVideoSource(argv+1, argc-1);
	// }

	if (ww::Config::source_type == ww::Config::Video) {
		vs = new ww::FilmVideoSource(ww::Config::video_file.c_str());
	}
	else {
		vs = new ww::ImageVideoSource(ww::Config::image_files);	
	}



	ww::ViewContent* vc = new ww::Slam(vs);
	ww::View view(vc);
	view.run();
	
	delete vs;
	delete vc;
	

	return 0;

}
