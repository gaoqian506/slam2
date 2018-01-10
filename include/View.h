

#ifndef __WW_VIEW_HEADER__
#define __WW_VIEW_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"
#include "Camera.h"
#include "Rectangle.h"
#include "Vectors.h"
#include <pthread.h>

namespace ww {



class View : public DisplayDelegate {

public:

	enum DisplayAspect {
		DisplaySpace = 0,
		DisplayImage, // 1
		// ...
	};

	View(ViewContent* vc);
	~View();
	
	void run();
	void tick();
	void display();
	void keyboard(unsigned char key,int x,int y);
	void special(int key,int x,int y);
	void mouse(int button, int state, int x, int y);
	void mouse_move(int x, int y);
	void passive_mouse_move(int x, int y);
	void start_content();
	void draw_content();
	void draw_image(Image* image);
	void draw_field(Image* image, Image* weight = NULL);
	void print_text(const char* str, int x, int y);
	void draw_cameras();
	void draw_points();
	void draw_mesh();
	void draw_camera_instance(Camera* camera, bool with_points = true);
	void draw_optical_flow(Image* of);
	ViewContent* content() { return m_content; }

	
	virtual void display_with(ViewContent* cv);
	
private:

	Rectangle get_scene_bounding_box(Camera** cameras, int count);
	

	ViewContent* m_content;
	pthread_t m_thread_id;
	DisplayAspect m_display_aspect;
	int m_display_index;
	int m_key_index;
	Vec3d m_trans_2d;
	unsigned int m_gl_texture;
	Vec2d m_pixel_pos;
	//char* m_pixel_info;
	Image* m_current_image;
	int m_mouse_pos[2];
	double m_vpp; //view per pixel in radian
	Vec16d m_view_matrix;
	int m_point_size;
	Vec2d m_epi_point;
	Image* m_image;
	Image* m_weight;
	int m_mouse_button;
	


};


}

#endif
