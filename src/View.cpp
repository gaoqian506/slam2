

#include "View.h"
#include "GlToolbox.h"
#include "SpaceToolbox.h"
#include "MatrixToolbox.h"
#include "Config.h"
#include <GL/freeglut.h>
#include <iostream>
#include <vector>
#include <assert.h>


namespace ww {

void* g_host = 0;

void g_display(void) {
	if (g_host) { ((View*)g_host)->display(); }
}

void g_idle(void) {

	if (g_host && ((View*)g_host)->content()->changed()) { ((View*)g_host)->display(); }
}

void g_keyboard(unsigned char key,int x,int y) {
	((View*)g_host)->keyboard(key, x, y);
}

void g_special(int key, int x, int y) {
	((View*)g_host)->special(key, x, y);
}

void g_mouse_func(int button, int state, int x, int y) {
	((View*)g_host)->mouse(button, state, x, y);
}

void g_mouse_move_func(int x, int y) {

	((View*)g_host)->mouse_move(x, y);
}

void g_passive_mouse_move_func(int x, int y) {

	((View*)g_host)->passive_mouse_move(x, y);
	//std::cout << x << "," << y << std::endl;
}



void* g_thread(void*) {

	std::cout << "g_thread" << std::endl;
	
	 ((View*)g_host)->start_content();
	
}


View::View(ViewContent* vc) {

	std::cout << "View::View" << std::endl;

	m_content = vc;
	vc->set_display_delegate(this);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(100, 100); 
	glutInitWindowSize(Config::win_size[0], Config::win_size[1]);
	glutCreateWindow("OpenGL 3D View");
	glutDisplayFunc(g_display);
	//glutIdleFunc(g_idle);
	glutKeyboardFunc(g_keyboard);
	glutSpecialFunc(g_special);
	glutMouseFunc(g_mouse_func);
	glutMotionFunc(g_mouse_move_func);
	glutPassiveMotionFunc(g_passive_mouse_move_func);
	glClearColor(0.233, 0.156, 0.343, 1);
	glEnable(GL_DEPTH_TEST);

	
	g_host = this;
	m_display_aspect = DisplayImage;
	m_display_index = 0;
	m_key_index = 0;
	m_trans_2d[0] = 1;
	//m_trans_2d[1] = 100;
	//m_trans_2d[2] = 100;
	m_image = NULL;
	m_weight = NULL;

	glGenTextures(1, &m_gl_texture);
	m_current_image = 0;
	m_point_size = 1;
	MatrixToolbox::identity(m_view_matrix);
	m_mouse_button = 0;


}

View::~View() {

	std::cout << "View::~View" << std::endl;
	m_content->stop();
	pthread_join(m_thread_id, 0);

}

void View::run() {

	std::cout << "View::run" << std::endl;

	

	pthread_create(&m_thread_id, NULL, g_thread, NULL);
	glutMainLoop();

}

void View::display() {

	std::cout << "View::display==========" << std::endl;
	//g_host = this;
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	switch (m_display_aspect) {
	case DisplaySpace:
		draw_content();
		glColor3d(0.35, 0.92, 0.72);
		print_text(m_content->pixel_info(m_pixel_pos, m_key_index), 5, 15);
		break;
	case DisplayImage:

		glColor3d(0.35, 0.92, 0.72);
		print_text(m_content->pixel_info(m_pixel_pos, m_key_index), 5, 15);
			
		glColor3d(0.72, 0.38, 0.49);
		m_image = m_content->get_debug_image(
			m_display_index, m_key_index, &m_weight);
		if (m_image && m_image->channels() == 2) {
			draw_field(m_image, m_weight);
		}
		else {
			draw_image(m_image); 
		}
		
		//glColor3d(0.92, 0.56, 0.37);
		//draw_optical_flow(m_content->get_optical_flow());

		break;
	}
	


	glutSwapBuffers();
}

void View::keyboard(unsigned char key,int x,int y) {

	double dist = 0.1;

	switch(key) {
	case 27:	// Esc
		glutLeaveMainLoop();
		break;			
	case 48:	// '0'
		m_display_aspect = DisplaySpace;
		glutPostRedisplay();
		break;
	case 49:	// '1'
	case 50:	// '2'
	case 51:	// '3'
	case 52:	// '4'
	case 53:	// '5'
	case 54:	// '6'
	case 55:	// '7'
	case 56:	// '8'
	case 57:	// '9'
		m_display_aspect = DisplayImage;
		if (m_display_index == key - 49) {
			Config::image_switch_list[m_display_index] =
			 !Config::image_switch_list[m_display_index];
		}
		else {
			m_display_index = key - 49;
		}
		// m_image = m_content->get_debug_image(
		// 	m_display_index, m_key_index, &m_weight);
		glutPostRedisplay();
		break;
	// case 'w':	// move forward
	// 	if (m_display_aspect == DisplaySpace) {
	// 		SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, -dist));
	// 	}
	// 	glutPostRedisplay();
	// 	break;
	case 's':	// move backward
		if (m_display_aspect == DisplaySpace) {	
			// SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, dist));
		}
		else {
			//Config::image_switch = !Config::image_switch;
			Config::image_switch_list[m_display_index] =
			 !Config::image_switch_list[m_display_index];			
		}
		glutPostRedisplay();
		break;
	// case 'a':	// move left
	// 	if (m_display_aspect == DisplaySpace) {
	// 		SpaceToolbox::translate(m_view_matrix, Vec3d(dist, 0, 0));
	// 	}
	// 	glutPostRedisplay();
	// 	break;
	// case 'd':	// move right
	// 	if (m_display_aspect == DisplaySpace) {
	// 		SpaceToolbox::translate(m_view_matrix, Vec3d(-dist, 0, 0));
	// 	}
	// 	glutPostRedisplay();
	// 	break;
	case 32:	// space
		//if (m_display_aspect == DisplaySpace) {
		//	MatrixToolbox::identity(m_view_matrix);
		//}
		//else {
		m_content->build((ViewContent::BuildFlag)(
			ViewContent::BuildReadFrame + 
			ViewContent::BuildOpticalFlow +
			//ViewContent::BuildEpipolar +
			ViewContent::BuildDepth +
			ViewContent::BuildKeyframe +
			ViewContent::BuildIterate +
			ViewContent::BuildSequence
		));
		glutPostRedisplay();
		//}	
		
		break;
	case '=':
		Config::field_skip--;
		if (Config::field_skip < 1) {
			Config::field_skip = 1;
		}
		glutPostRedisplay();		
		break;		
	case '-':
		Config::field_skip++;
		glutPostRedisplay();		
		break;	
	case 'b':	// key for break
		key = 'b';
		break;
	}
}

void View::special(int key,int x,int y) {

	double dist = 0.1;
	switch(key) {
	case GLUT_KEY_F1:

		m_content->build((ViewContent::BuildFlag)(
			ViewContent::BuildReadFrame + 
			//ViewContent::BuildOpticalFlow +
			//ViewContent::BuildEpipolar +
			ViewContent::BuildKeyframe
			//ViewContent::BuildIterate
		));
		glutPostRedisplay();
		break;
	case GLUT_KEY_F2:
		//m_content->func_manualy(1);
		m_content->build((ViewContent::BuildFlag)(
			//ViewContent::BuildReadFrame + 
			ViewContent::BuildOpticalFlow
			//ViewContent::BuildEpipolar +
			//ViewContent::BuildKeyframe +
			//ViewContent::BuildIterate
		));	
		display();
		//glutPostRedisplay();
		break;
	case GLUT_KEY_F3:
		//m_content->func_manualy(2);
		m_content->build((ViewContent::BuildFlag)(
			//ViewContent::BuildReadFrame + 
			//ViewContent::BuildOpticalFlow
			ViewContent::BuildEpipolar
			//ViewContent::BuildKeyframe +
			//ViewContent::BuildIterate
		));			
		glutPostRedisplay();
		break;
	case GLUT_KEY_F4:
		m_content->build((ViewContent::BuildFlag)(
			//ViewContent::BuildReadFrame + 
			//ViewContent::BuildOpticalFlow
			ViewContent::BuildDepth
			//ViewContent::BuildKeyframe +
			//ViewContent::BuildIterate
		));	
		glutPostRedisplay();
		break;
	case GLUT_KEY_F5:
		m_content->func_manualy(4);
		glutPostRedisplay();
		break;
	case GLUT_KEY_F6:
		m_content->func_manualy(5);
		glutPostRedisplay();
		break;
	case GLUT_KEY_F10:
		m_content->build(ViewContent::BuildReadFrame);
		glutPostRedisplay();
		break;
	case GLUT_KEY_UP:
		if (m_display_aspect == DisplaySpace) {
			m_point_size ++;
		}
		else {
			//m_display_index++;
			int viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);
			GlToolbox::zoom_screen(viewport[2]>>1, viewport[3]>>1, 1.25, m_trans_2d);
			// m_image = m_content->get_debug_image(
			// 	m_display_index, m_key_index, &m_weight);			
		}
		glutPostRedisplay();
		break;
	case GLUT_KEY_DOWN:
		if (m_display_aspect == DisplaySpace) {
			m_point_size--;
			if (m_point_size < 1) { m_point_size = 1; }
		}
		else {
			// m_display_index = m_display_index ? m_display_index-1 : 0;
			int viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);
			//std::cout << (viewport[2]>>1) << " " << (viewport[3]>>1) << std::endl;
			GlToolbox::zoom_screen(viewport[2]>>1, viewport[3]>>1, 0.8, m_trans_2d);
			// m_image = m_content->get_debug_image(
			// 	m_display_index, m_key_index, &m_weight);
		}
		glutPostRedisplay();
		break;
	case GLUT_KEY_LEFT:
		if (m_display_aspect == DisplaySpace) {
			SpaceToolbox::translate(m_view_matrix, Vec3d(dist, 0, 0));
		}
		else {
			//m_trans_2d[0] *= 0.8;
			m_key_index ? m_key_index-- : m_key_index;
		}
		glutPostRedisplay();
		break; 
	case GLUT_KEY_RIGHT:
		if (m_display_aspect == DisplaySpace) {
			SpaceToolbox::translate(m_view_matrix, Vec3d(-dist, 0, 0));
		}
		else {
			
			m_key_index++;
			//m_trans_2d[0] *= 1.25;
		}
		glutPostRedisplay();
		break;	}
}

void View::mouse(int button, int state, int x, int y) {

	double dist = 0.25;
	m_mouse_pos[0] = x;
	m_mouse_pos[1] = y;	

	//std::cout << x << " " << y << std::endl;

	switch(button) {

	case GLUT_LEFT_BUTTON:
		m_mouse_button = 1;
		break;
	case GLUT_RIGHT_BUTTON:
		m_mouse_button = 2;
		break;		
	case 3:
		if (m_display_aspect == DisplaySpace) {
			SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, -dist));
		}
		else {
			GlToolbox::zoom_screen(x, y, 1.25, m_trans_2d);
		}
		glutPostRedisplay();
		break;
	case 4:
		if (m_display_aspect == DisplaySpace) {
			SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, dist));
		}
		else {
			GlToolbox::zoom_screen(x, y, 0.8, m_trans_2d);
		}
		glutPostRedisplay();
		break;
	}

	if (state ==  GLUT_UP) { m_mouse_button = 0; }



}

void View::mouse_move(int x, int y) {

	int dx = x-m_mouse_pos[0];
	int dy = y-m_mouse_pos[1];

	if (m_display_aspect == DisplaySpace/* && abs(dx) < 20 && abs(dy) < 20*/) {
		Vec9d dR = SpaceToolbox::make_rotation(-dy*m_vpp, dx*m_vpp);
		SpaceToolbox::rotate(m_view_matrix, dR);
		if (m_mouse_button == 1) {
			SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, -0.02));	
		}
		else if (m_mouse_button == 2) {
			SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, +0.02));	
		}		
		
		glutPostRedisplay();
	}

	m_mouse_pos[0] = x;
	m_mouse_pos[1] = y;

}

void View::passive_mouse_move(int x, int y) {


	//std::cout << "passive_mouse_move:" << dx << "," << dy << std::endl;

	if (m_current_image) {
		// m_pixel_pos = GlToolbox::screen_to_image(
		// 	x, y, m_trans_2d[0], 
		// 	m_current_image->width(), 
		// 	m_current_image->height()
		// );
	std::cout << "View::draw_content" << std::endl;

		m_pixel_pos = GlToolbox::screen_to_image(
			x, y, m_trans_2d, 
			m_current_image->width(), 
			m_current_image->height()
		);
		glutPostRedisplay();
	}

}

void View::draw_content() {


	std::cout << "View::draw_content" << std::endl;
		
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	gluPerspective(60.0, (double)viewport[2]/viewport[3], 0.1, 1000);
	m_vpp = 120.0/viewport[3]/180*3.1415926535898;
	//gluPerspective(60, 1, 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRotated(180, 1, 0, 0);
	glMultTransposeMatrixd(m_view_matrix.val);

	Camera** cameras = m_content->get_cameras();
	int camera_count = m_content->get_camera_count();

		
	glColor3d(0.87,0.623,3.222);

	for (int i = 0; i < camera_count; i++) {
		draw_camera_instance(cameras[i], true);
	}

	Camera* current = m_content->get_current_frame();
	if (current) {
		glColor3d(0.57,4.623,7.222);
		draw_camera_instance(current, false);
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();	
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

}

void View::draw_image(Image* image) {

	if(!image) { return; }
	//glScaled(10, 10, 10);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	GlToolbox::orthogonal_pixel(GlToolbox::Center);
	glTranslated(m_trans_2d[1], m_trans_2d[2], 0);
	glScaled(m_trans_2d[0], m_trans_2d[0], 1);
	//std::cout << m_trans_2d[0] << " " << m_trans_2d[1] << " " << m_trans_2d[2] << std::endl;

	int w = image->width();
	int h = image->height();
	
	GlToolbox::setup_texture(m_gl_texture, image);
	glBindTexture(GL_TEXTURE_2D, m_gl_texture);
	glEnable(GL_TEXTURE_2D);

	glBegin(GL_QUADS);
	glTexCoord2i(0, 0); glVertex2d(-w/2, -h/2);
	glTexCoord2i(0, 1); glVertex2d(-w/2, +h/2);
	glTexCoord2i(1, 1); glVertex2d(+w/2, +h/2);
	glTexCoord2i(1, 0); glVertex2d(+w/2, -h/2);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	// glColor3d(0.233, 0.753, 0.777);
	// glBegin(GL_LINE_LOOP);
	// glVertex2d(-w/2, -h/2);
	// glVertex2d(-w/2, +h/2);
	// glVertex2d(+w/2, +h/2);
	// glVertex2d(+w/2, -h/2);
	// glEnd();
	int u = (int)m_pixel_pos[0];
	int v = (int)m_pixel_pos[1];
	double m[2] = { -w*0.5+0.5, -h*0.5+0.5 };

	glLineWidth(2);
	Camera* camera = m_content->get_current_frame();
	if (camera) {
		Vec2d ep = camera->image_epi_point();
		glColor3d(1, 1, 0);
		glBegin(GL_LINES);
		glVertex2d(u+m[0], v+m[1]);
		glVertex2d(ep[0]+m[0], ep[1]+m[1]);
		glEnd();
	}


	glLineWidth(1);		

	m_current_image = image;

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

}

void View::draw_field(Image* field, Image* weight/* = NULL*/) {


	if(!field) { return; }

	m_current_image = field;
	assert(field->channels() == 2 && 
		field->type() == Image::Float32);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	
	GlToolbox::orthogonal_pixel(GlToolbox::Center);
	glTranslated(m_trans_2d[1], m_trans_2d[2], 0);
	glScaled(m_trans_2d[0], m_trans_2d[0], 1);
	int w = field->width();
	int h = field->height();

	int skip = Config::field_skip, id;
	Vec2f mv;
	double m[2] = { -w*0.5+0.5, -h*0.5+0.5 }, t;

	for (int u = 0; u < w; u+=skip) {

		for (int v = 0; v < h; v+=skip) {
			mv = ((Vec2f*)field->data())[v*w+u];
			if (weight) {
				t = ((float*)weight->data())[v*w+u];
			}
			else { t = 1; }
			glColor3d((1-t), t, 0);
			glBegin(GL_LINES);
			glVertex2d(u+m[0], v+m[1]);
			glVertex2d(u+m[0]+mv[0], v+m[1]+mv[1]);
			glEnd();
		}
	}

	if (m_pixel_pos[0] >= 0 && m_pixel_pos[0] < w &&
		m_pixel_pos[1] >= 0 && m_pixel_pos[1] < h) {

		int u = (int)m_pixel_pos[0];
		int v = (int)m_pixel_pos[1];
		mv = ((Vec2f*)field->data())[v*w+u];

		glLineWidth(4);
		glColor3d(0, 0, 1);
		glBegin(GL_LINES);
		glVertex2d(u+m[0], v+m[1]);
		glVertex2d(u+m[0]+mv[0], v+m[1]+mv[1]);
		glEnd();

		glLineWidth(2);
		Camera* camera = m_content->get_current_frame();
		if (camera) {
			Vec2d ep = camera->image_epi_point();
			glColor3d(1, 1, 0);
			glBegin(GL_LINES);
			glVertex2d(u+m[0], v+m[1]);
			glVertex2d(ep[0]+m[0], ep[1]+m[1]);
			glEnd();
		}


		glLineWidth(1);		

	}



	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

}

void View::print_text(const char* str, int x, int y) {

	if (!str) { return; }

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	GlToolbox::orthogonal_pixel(GlToolbox::TopLeft);
	glRasterPos2f(x, y);

	while(str[0]) {
		if (str[0] == '\n') {
			y += 20;
			glRasterPos2f(x, y);
		}
		else {
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[0]);
		}


		str++;
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();


}

void View::draw_cameras() {
	std::cout << "View::draw_cameras" << std::endl;
	assert(0);
	Camera** cameras = m_content->get_cameras();
	int camera_count = m_content->get_camera_count();
	
	Rectangle bb = get_scene_bounding_box(cameras, camera_count);
	
	if (bb.width == 0 || bb.height == 0) {
		return;
	}
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	
	GlToolbox::othorgonal(bb);
	
	//glColor3d(1, 1, 1);

		
	glColor3d(0.87,0.623,3.222);
	
	//glBegin(GL_POINTS);
	//glVertex3d(0, 0, 0);
	//glVertex3d(1, 0, 1);
	//glEnd();

	for (int i = 0; i < camera_count; i++) {
		draw_camera_instance(cameras[i]);
	}

	Camera* current = m_content->get_current_frame();
	if (current) {
		glColor3d(0.57,4.623,7.222);
		draw_camera_instance(current);
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();	
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	
}

void View::draw_points() {
	std::cout << "View::draw_points" << std::endl;
}

void View::draw_mesh() {
	std::cout << "View::draw_mesh" << std::endl;
}

void View::draw_camera_instance(Camera* camera, bool with_points /*=true*/) {

	std::cout << "View::draw_camera_instance" << std::endl;
	if (!camera->gray) { return; }
	if (Config::method == Config::Lsd3) {
		GlToolbox::transform_to(camera->pos, camera->rotation, false);
	}
	else {
		GlToolbox::transform_to(camera->pos, camera->rotation, true);
	}

	
	int width = camera->gray->width();
	int height = camera->gray->height();
	std::vector<Vec3d> image_corners;
	image_corners.push_back(Vec3d(0, 0, 1));
	image_corners.push_back(Vec3d(width, 0, 1));
	image_corners.push_back(Vec3d(width, height, 1));
	image_corners.push_back(Vec3d(0, height, 1));
	//.. push other three corners
	std::vector<Vec3d> sensor_corners = SpaceToolbox::unproject(camera->intrinsic, image_corners);

	
	glBegin(GL_LINES);
	for (std::vector<Vec3d>::const_iterator itr = sensor_corners.begin(); itr != sensor_corners.end(); itr++) {
		glVertex3d(0, 0, 0);
		glVertex3dv(itr->val);
	}
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	for (std::vector<Vec3d>::const_iterator itr = sensor_corners.begin(); itr != sensor_corners.end(); itr++) {
		glVertex3dv(itr->val);
	}
	glEnd();
	
	if (with_points && camera->points) {
		glPointSize(m_point_size);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(4, GL_FLOAT, 0, camera->points->data());
		glColorPointer(3, GL_UNSIGNED_BYTE, 0, camera->original->data());
		glDrawArrays(GL_POINTS, 0, width*height);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);
	}

}

void View::draw_optical_flow(Image* of) {

	assert(0);
	if(!of) { return; }

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	
	GlToolbox::orthogonal_pixel(GlToolbox::Center);
	glTranslated(m_trans_2d[1], m_trans_2d[2], 0);
	glScaled(m_trans_2d[0], m_trans_2d[0], 1);
	int w = of->width();
	int h = of->height();

	int skip = Config::of_skip, id;
	Vec2f mv;
	double m[2] = { -w*0.5+0.5, -h*0.5+0.5 };

	for (int u = 0; u < w; u+=skip) {

		for (int v = 0; v < h; v+=skip) {
			mv = ((Vec2f*)of->data())[v*w+u];
			glBegin(GL_LINES);
			glVertex2d(u+m[0], v+m[1]);
			glVertex2d(u+m[0]-mv[0], v+m[1]-mv[1]);
			glEnd();
		}
	}

	if (m_pixel_pos[0] >= 0 && m_pixel_pos[0] < w &&
		m_pixel_pos[1] >= 0 && m_pixel_pos[1] < h) {

		int u = (int)m_pixel_pos[0];
		int v = (int)m_pixel_pos[1];
		mv = ((Vec2f*)of->data())[v*w+u];

		glLineWidth(2);
		glColor3d(1, 0, 0);
		glBegin(GL_LINES);
		glVertex2d(u+m[0], v+m[1]);
		glVertex2d(u+m[0]-mv[0], v+m[1]-mv[1]);
		glEnd();
		glLineWidth(1);
	}
	

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();


}

void View::start_content() {

	std::cout << "View::start_content" << std::endl;
	m_content->start();

}

void View::display_with(ViewContent* cv) {

	std::cout << "View::display_with" << std::endl;
	display();
	//glutPostRedisplay();
	

	
}

Rectangle View::get_scene_bounding_box(Camera** cameras, int count) {

	std::cout << "View::get_scene_bounding_box" << std::endl;

	
	double left = 0, right = 0, bottom = 0, top = 0;
	double temp;
	

	for (int i = 0; i < count; i++) {
	
		temp = cameras[i]->pos[0]-cameras[i]->radius;
		if (left > temp) { left = temp; }
		
		temp = cameras[i]->pos[0]+cameras[i]->radius;
		if (right < temp) { right = temp; }
	
		temp = cameras[i]->pos[2]-cameras[i]->radius;
		if (bottom > temp) { bottom = temp; }
		
		temp = cameras[i]->pos[2]+cameras[i]->radius;
		if (top < temp) { top = temp; }
	}

	return Rectangle(left, bottom, right-left, top-bottom);
}


} // namespace

/****************************************



	double dist = 0.1;

	switch(key) {
	case 48:	// '0'
		m_display_aspect = DisplaySpace;
		glutPostRedisplay();
		break;
	case 49:	// '1'
		m_display_aspect = DisplayImage;
		glutPostRedisplay();
		break;
	case 'w':	// move forward
		SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, -dist));
		glutPostRedisplay();
		break;
	case 's':	// move backward
		SpaceToolbox::translate(m_view_matrix, Vec3d(0, 0, dist));
		glutPostRedisplay();
		break;
	case 'a':	// move forward
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
			m_content->push_manauly();
		}
		else {
			SpaceToolbox::translate(m_view_matrix,
 Vec3d(dist, 0, 0));
		}
		glutPostRedisplay();
		break;
	case 'd':	// move backward
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
			m_content->func_manualy(key-'a');
		}
		else {
			SpaceToolbox::translate(m_view_matrix, Vec3d(-dist, 0, 0));
		}
		glutPostRedisplay();
		break;
	case 'b':	// manualy function 1
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
			m_content->func_manualy(key-'a');
		}
		glutPostRedisplay();
		break;
	case 'c':	// manualy function 2
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
			m_content->func_manualy(key-'a');
		}
		glutPostRedisplay();
		break;
	case 'e':	// manualy function 3
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
			m_content->func_manualy(key-'a');
		}
		glutPostRedisplay();
		break;
	}


	if (m_display_aspect == DisplaySpace) {

		switch(key) {

		}
	}
	else {
		switch(key) {
		case 'a':	// push manauly

			glutPostRedisplay();
			break;
		case 'b':	// manualy function 1
		case 'c':	// manualy function 2
		case 'd':	// manualy function 3
		case 'e':	// manualy function 3
			m_content->func_manualy(key-'a');
			glutPostRedisplay();
			break;
		}
	}




	glRotated(180, 1, 0, 0);
	glTranslated(0.1, 0.1, 0);


	glColor3d(1, 1, 1);
	glBegin(GL_LINE_LOOP);
	glVertex3d(-0.5, -0.5, 1.5);
	glVertex3d(-0.5, +0.5, 1.5);
	glVertex3d(+0.5, +0.5, 1.5);
	glVertex3d(+0.5, -0.5, 1.5);
	glEnd();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();	
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	return;


	glColor3d(1, 1, 1);
	glBegin(GL_LINE_LOOP);
	glVertex3d(-0.5, -0.5, 0);
	glVertex3d(-0.5, +0.5, 0);
	glVertex3d(+0.5, +0.5, 0);
	glVertex3d(+0.5, -0.5, 0);
	glEnd();


		case 50:	// DisplayImage

			break;

			m_content->func_manualy(1);

	//glutMouseWheelFunc(g_mouse_wheel);
	//glClearColor(0.233, 0.156, 0.43, 1);


	glColor3d(1, 0, 0);
	glBegin(GL_LINE_LOOP);
	glVertex3d(-200, -200, 0);
	glVertex3d(-200, +200, 0);
	glVertex3d(+200, +200, 0);
	glVertex3d(+200, -200, 0);
	glEnd();

				//setup_eye_3d();
		//draw_image(m_content->get_debug_image(m_display_index));

		//glRotatef(-90, 1, 0, 0);
		//draw_cameras();
		//draw_points();
		//draw_mesh();

	glBegin(GL_POINTS);
	glVertex3d(0, 0, 0);
	glEnd();


	std::cout << "View::display" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT);
	//glBegin(GL_POINTS);
	//glEnd();
	glRectf(-0.5f,-0.5f,0.5f,0.5f);



void View::tick() {

	std::cout << "View::tick" << std::endl;

	//m_content->tick();
	display();



}



int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowPosition(0, 0); 
    glutInitWindowSize(300, 300);
    glutCreateWindow("OpenGL 3D View");
    glutDisplayFunc(display);
    glutMainLoop();

    return 0;
}


#include "controller.h"
#include <iostream>			// cout
#include <stdio.h>			// fopen
#include "opencv2/highgui/highgui.hpp"	//cv::imread
#include "config.h"
#include "toolbox.h"

huma::Controller* g_host;


namespace huma {

	void main_loop(void) {

		g_host->display();
	}

	void mouse_func(int button, int state, int x, int y) {
		g_host->mouse(button, state, x, y);

	}
	void mouse_move_func(int x,int y) {
		g_host->mouse_move(x, y);

	}
	void passive_mouse_move_func(int x, int y) {
		g_host->passive_mouse_move(x, y);
	}
	void keyboard_func(unsigned char key,int x,int y) {
		g_host->keyboard(key, x, y);
	}

	void Special_keyboard_func(int key, int x, int y) {
		g_host->Specialkeyboard(key, x, y);
	}

	void menu_func(int id) {
		g_host->menu(id);
	}

	void reshape_func(int w, int h) {
		g_host->reshape(w, h);
	}
//----------------------------------------------------------------

	Controller::Controller() {

		g_host = this;
		init_glut(0, 0);
		_cmd_cursor = 0;
		_cmd_buffer[0] = 0;
		_purpose = Normal;
		memset(_rectangle, 0, sizeof(_rectangle));
		memset(_image_trans, 0, sizeof(_image_trans));
		memset(_image_ids, 0, sizeof(_image_ids));
		_image_trans[0] = 1;
		_pixel_value[0] = 0;
		command("test");
		Config::instance()->min_pyramid_size();

	}

	void Controller::run() {
		glutMainLoop();
	}

	void Controller::display() {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		draw_image();
		//draw_rectangle();
		//draw_polygon();
		
		int x = 18, y = 15;
		print_text(_cmd_buffer, x, y);
		print_text(_pixel_value, x, y);


		glutSwapBuffers();
	}

	void Controller::menu(int id) {
		set_purpose((Purpose)id);
	}

	void Controller::keyboard(unsigned char key,int x,int y) {

		switch(key) {

		//Enter
		case 13:	
			command(_cmd_buffer);
			_cmd_cursor = 0;
			_cmd_buffer[_cmd_cursor] = 0;

		break;
		//Backspace
		case 8:
			_cmd_cursor = _cmd_cursor > 0 ? _cmd_cursor-1 : 0;
			_cmd_buffer[_cmd_cursor] = 0;
		break;
		//Zero
		case 48:
			_image_ids[0] = 0;
			_image_ids[1] = 0;
			select_image();
		break;
		//Escape...
		case 27:
			set_purpose(Normal);
		break;
		default:
			_cmd_buffer[_cmd_cursor++] = key;
			_cmd_buffer[_cmd_cursor] = 0;

		}

		display();
	}

	void Controller::Specialkeyboard(int key, int x, int y) {
		switch(key) {
		case GLUT_KEY_LEFT:
			_image_ids[0]--;
			if (_image_ids[0] < 0) { _image_ids[0] = 0; }
			_image_ids[1] = 0;
		break;
		case GLUT_KEY_RIGHT:

			//if (check_layer(_image_ids[0]+1, _image_ids[1])) {
				_image_ids[0]++;
				_image_ids[1] = 0;
			//}else {
			//	std::cout<< "No this layer!" << "\n";
			//}
		break;
		case GLUT_KEY_UP:
			//if (check_layer(_image_ids[0], _image_ids[1]+1)) {
				_image_ids[1]++;
			//}else {
			//	std::cout<< "No this layer!" << "\n";
			//}
		break;
		case GLUT_KEY_DOWN:
			_image_ids[1]--;
			if (_image_ids[1] < 0) { _image_ids[1] = 0; }
		break;
		}
		select_image();
	}

	void Controller::mouse(int button, int state, int x, int y) {

		if (button == 3 || button == 4) {
			double ds = button == 3 ? 1.25 : 0.8;
			double dx = x - _win_size[0]*0.5;
			double dy = _win_size[1]*0.5 - y;
			_image_trans[0] *= ds;
			_image_trans[1] = ds*(_image_trans[1]-dx)+dx;
			_image_trans[2] = ds*(_image_trans[2]-dy)+dy;
		}
		
		switch (_purpose) {

		case PreRectangle:
			if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
				cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
				_rectangle[0] = p[0]; _rectangle[1] = p[1];
				_rectangle[2] = _rectangle[0]; _rectangle[3] = _rectangle[1]; 
				set_purpose(DoingRectangle);
			}
		break;
		case PrePolygon:
			if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
				if (_polygon.size() > 3) {
					cv::Vec2d p = image_to_screen(_polygon.front());
					if (cv::norm(p - cv::Vec2d(x, y)) < 10) {
						_polygon.back() = _polygon.front();
						set_purpose(Normal);
						break;
					}
				}
				cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
				_polygon.push_back(p);
			}
		break;
		case DoingRectangle:
			if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
				set_purpose(Normal);
			}
		break;
		case Normal:

		break;
			
		}
		display();

	}

	void Controller::mouse_move(int x,int y) {

		if (!_image.empty()) {

			cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
			p[0] = (_image.cols>>1)+p[0];
			p[1] = (_image.rows>>1)-p[1];

                        if (p[1] >=0 && p[1] < _image.rows && p[0] >= 0 && p[0] < _image.cols) {

                                if (_image.type() == CV_32F && _image.channels() == 1) {
                                        sprintf(_pixel_value, "%.2f,%.2f:%f", p[0], p[1] , _image.at<float>((int)p[1], (int)p[0]));
                                }
                                else if (_image.type() == CV_8U && _image.channels() == 1) {
                                        sprintf(_pixel_value, "%.2f,%.2f:%d", p[0], p[1] , _image.at<unsigned char>((int)p[1], (int)p[0]));
                                }
                                else if (_image.type() == CV_8S && _image.channels() == 1) {
                                        sprintf(_pixel_value, "%.2f,%.2f:%d", p[0], p[1] , _image.at<char>((int)p[1], (int)p[0]));
                                }
			}
		}

		switch (_purpose) {

		case DoingRectangle:
			cv::Vec2d p = screen_to_image(cv::Vec2d(x, y));
			_rectangle[2] = p[0]; _rectangle[3] = p[1];
		break;
			
		}
		display();
		
	}

	void Controller::passive_mouse_move(int x,int y) {

		switch (_purpose) {
		case PrePolygon:
			_polygon.back() = screen_to_image(cv::Vec2d(x, y));
			display();
		break;
			
			
		}

	}

	void Controller::reshape(int w, int h) {
		_win_size[0] = w;
		_win_size[1] = h;
		glViewport(0, 0, w, h);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-w*0.5, w*0.5, -h*0.5, h*0.5, -1, 1);
	}


//-------------------------------------------------------------------------------
	void Controller::init_glut(int argc, char** argv) {

		_gl_texture = 0;
		glutInit(&argc, argv);


		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
		glutInitWindowSize(800, 600);
		glutCreateWindow("mahu");
		glutDisplayFunc(main_loop); //glutIdleFunc glutDisplayFunc
		glutKeyboardFunc(keyboard_func);
		glutSpecialFunc(Special_keyboard_func);
		glutMouseFunc(mouse_func);
		glutMotionFunc(mouse_move_func);
		glutPassiveMotionFunc(passive_mouse_move_func);
		glutReshapeFunc(reshape_func);
		//glutSetIconTitle("favicon.ico");
		//glutSetWindowTitle("favicon.ico");
		glClearColor(0.3, 0.3, 0.3, 1);

		//assert(glewInit() == GLEW_OK);

		// menus
		 int menu;
		 menu = glutCreateMenu(menu_func);
		 glutAddMenuEntry("Polygon", PrePolygon);
		 glutAddMenuEntry("Rectangle", PreRectangle);
		 glutAddMenuEntry("ResetImage", ResetImage);
		 glutAttachMenu(GLUT_RIGHT_BUTTON);

		//std::cout << "controller's glut initialized.\n";

	}

	void Controller::setup_gl_texture( cv::Mat image ) {
		if (image.empty()) { return; }
		image.copyTo(_image);


		if (_gl_texture == 0) { glGenTextures(1, &_gl_texture);	}

		glBindTexture(GL_TEXTURE_2D, _gl_texture);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

		double min, max, scale;
		cv::minMaxIdx(image, &min, &max);
		scale = 1./(max-min);
		_image.convertTo(image, 
			CV_32FC(image.channels()), scale, -min/(max-min));				

		unsigned int internalFormat = 0, format = 0, type = 0;

		if (image.type() == CV_32F)
		{ internalFormat = GL_LUMINANCE; format = GL_LUMINANCE; type = GL_FLOAT; }
		else if (image.type() == CV_32FC3)
		{ internalFormat = GL_RGB; format = GL_BGR; type = GL_FLOAT; }
		else if (image.type() == CV_32FC4)
		{ internalFormat = GL_RGBA; format = GL_RGBA; type = GL_FLOAT; }
		else if (image.type() == CV_8UC3)
		{ internalFormat = GL_RGB; format = GL_RGB; type = GL_UNSIGNED_BYTE; }
		else { assert(0); }

		int w = image.cols;
		int h = image.rows;
		_image_size[0] = w;
		_image_size[1] = h;

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, type, image.data);

		glBindTexture(GL_TEXTURE_2D, 0);
	}



	
	void Controller::draw_image() {

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		
		if(_gl_texture){

			glTranslated(_image_trans[1], _image_trans[2], 0);
			glScaled(_image_trans[0], _image_trans[0], 1); 

			glBindTexture(GL_TEXTURE_2D, _gl_texture);
			glEnable(GL_TEXTURE_2D);

			glBegin(GL_QUADS);

			glTexCoord2f(0.0f, 1.0f); glVertex2d(-0.5*_image_size[0], -0.5*_image_size[1]);
			glTexCoord2f(1.0f, 1.0f); glVertex2d(+0.5*_image_size[0], -0.5*_image_size[1]);
			glTexCoord2f(1.0f, 0.0f); glVertex2d(+0.5*_image_size[0], +0.5*_image_size[1]);
			glTexCoord2f(0.0f, 0.0f); glVertex2d(-0.5*_image_size[0], +0.5*_image_size[1]);


			glEnd();

			glDisable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, 0);

		}

		glColor3d(1,0,0);
		glBegin(GL_LINE_LOOP);
		glVertex2d(_rectangle[0], _rectangle[1]);
		glVertex2d(_rectangle[2], _rectangle[1]);
		glVertex2d(_rectangle[2], _rectangle[3]);
		glVertex2d(_rectangle[0], _rectangle[3]);
		glEnd();

		glColor3d(1,0,0);
		glBegin(GL_LINE_STRIP);
		for (std::vector<cv::Vec2d>::iterator itr = _polygon.begin(); 
			itr != _polygon.end(); itr++) {
			glVertex2dv(itr->val);
			

		}
		glEnd();

		glPopMatrix();

	}

	void Controller::print_text(const char* text, int& x, int& y) {

		glColor3d(0,1,0);
		glRasterPos2d(x-_win_size[0]*0.5,_win_size[1]*0.5-y);
		for (int i = 0; i < strlen(text); i++) {
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, text[i]);
		}

		y += 10;

	}

	void Controller::command(const char* cmd) {

		std::cout << "\ncommand:" << cmd << "\n";

		if (int k = Toolbox::leftcmp("open ", cmd)) {
			open_image(cmd+k);
			strcpy(_image_name, cmd+k);
			//select_image();
		}
		else if (strcmp("test", cmd) == 0) {

			if(FILE *fp = fopen("command/commands.cmd", "r")) {
				char buffer[1024];
				while(fgets(buffer, 1024, fp)) {
					if (Toolbox::leftcmp("#", buffer)) { continue; }
					buffer[strlen(buffer)-1] = 0;
					//std::cout << "test command:" << buffer << "\n";
					command(buffer);

				}
				fclose(fp);
			}
			
		}
		else if (strcmp("exit", cmd) == 0 || strcmp("q", cmd) == 0) {
			exit(0);
		}
		else if (int k = Toolbox::leftcmp("go ", cmd)) {//"object "
			char cmd2[256] = "object 1";
			//sprintf(cmd2, "object %s", cmd+k);
			int n = atoi(cmd+k);
			std::cout << "will train " << n << "times!\n";
			for (int i = 0; i < n; i++) {
				std::cout << "step:------------------" << i << "----------------\n";
				command(cmd2);
				command("test");


			}
		}
		else if (int k = Toolbox::leftcmp("object ", cmd)) {//"object "
			std::vector<cv::Vec2d> poly = center_to_corner(_polygon);
			_detector.set(cmd+k, poly);
		}
		else {
			std::cout << "invalid command:" << cmd << "!\n";
		}

		select_image();

	}

	void Controller::open_image(const char* name) {

		cv::Mat img = cv::imread(name,CV_LOAD_IMAGE_COLOR);
		if(!img.empty()) {
			_detector.set_image(img);
			select_image();
		}else {
			std::cout<< "open image failed! image name:" << name << ".\n";
		}
	}

	void Controller::set_purpose(Purpose p) {

		switch (p) {
		case Normal:
			glutSetCursor(GLUT_CURSOR_RIGHT_ARROW);
		break;
		case PreRectangle:
			memset(_rectangle, 0, sizeof(_rectangle));
			glutSetCursor(GLUT_CURSOR_CROSSHAIR);
		break;
		case PrePolygon:
			_polygon.clear();
			_polygon.push_back(cv::Vec2d(0, 0));
			glutSetCursor(GLUT_CURSOR_CROSSHAIR);
		break;
		case ResetImage:
			memset(_image_trans, 0, sizeof(_image_trans));
			_image_trans[0] = 1;
			p = Normal;
		break;


		}
		//std::cout << "purpose:" << p << ".\n";
		_purpose = p;
		display();
	}

	cv::Vec2d Controller::screen_to_image(cv::Vec2d p) {

		double a = p[0] - _win_size[0]*0.5;
		double b = _win_size[1]*0.5 - p[1];

		return cv::Vec2d(
				(a-_image_trans[1])/_image_trans[0],
				(b-_image_trans[2])/_image_trans[0]
			);

	}

	cv::Vec2d Controller::image_to_screen(cv::Vec2d p) {

		double a = _image_trans[0] * p[0] + _image_trans[1];
		double b = _image_trans[0] * p[1] + _image_trans[2];
		return cv::Vec2d(a+_win_size[0]*0.5, _win_size[1]*0.5-b);
	}

	std::vector<cv::Vec2d> Controller::center_to_corner(std::vector<cv::Vec2d> ps) {


		std::vector<cv::Vec2d> out;
		for (std::vector<cv::Vec2d>::iterator itr = ps.begin();
			itr != ps.end(); itr++) {
			out.push_back(
				cv::Vec2d(
					(_image_size[0]>>1)+itr->val[0],
					(_image_size[1]>>1)-itr->val[1]
				)
			);
		}
		return out;
	}

	void Controller::select_image() {
		
		std::cout << "debug_image(" << _image_ids[0] <<"," << _image_ids[1] << "):";

		Layer* layer = _detector.get_layer(_image_ids[0]);
		if (layer) {
			std::cout << layer->name();
			cv::Mat image = layer->get_image(_image_ids[1]);
			if (!image.empty()) {
				setup_gl_texture(image);
			}
		}

		std::cout << "\n";

		display();

	}

	bool Controller::check_layer(int idx1, int idx2) {

		Layer* layer = _detector.get_layer(idx1);
		if (layer) {

			cv::Mat image = layer->get_image(idx2);
			if (!image.empty()) {
				return true;
			}
		}

		return false;

	}
}

*/





