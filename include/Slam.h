
#ifndef __WW_SLAM_HEADER__
#define __WW_SLAM_HEADER__


#include "ViewContent.h"
#include "VideoSource.h"
#include "Vectors.h"
#include <vector>

#define MAX_STATIC_CAMERA_COUNT 100
#define MAX_PIXEL_INFO 2048

namespace ww {

class Slam : public ViewContent {

public:
	Slam(VideoSource* vs);

	virtual void start();
	virtual void stop();
	virtual Camera** get_cameras();
	virtual int get_camera_count();
	virtual Camera* get_current_frame();
	virtual bool changed();
	virtual Image* get_debug_image(int iid, int kid, Image** weight = 0);
	virtual void push_manauly();
	virtual void func_manualy(int idx);
	virtual char* pixel_info(const Vec2d& u, int kid);
	virtual Image* get_optical_flow();
	virtual void build(BuildFlag flag = BuildAll);


private:

	void initialize(Image* image);
	void push(Image* image);
	void push_epi(Image* image);
	void preprocess(Image* image);
	void make_epi_line(Camera* camera);
	void update_pose();
	void calc_epipolar();
	void calc_epi_trans();
	void update_keyframe(Image* image);
	void update_map();

	void prepare_residual();
	void prepare_residual_epi();
	void prepare_residual_entropy2(bool use_trans = false);
	void prepare_residual_lsd2();
	void prepare_residual_lsd3();
	void prepare_residual_lsd4();
	void prepare_residual_epi2(bool with_trans = false);
	void prepare_residual_of1();
	void prepare_residual_of2();
	void prepare_residual_gof1();
	Vec3d calc_delta_t();
	Vec3d calc_delta_r();
	Vec3d calc_epi_point();
	Vec3d calc_epi_dr();
	Vec3d calc_dr_entropy();
	Vec3d calc_dr_entropy2();
	Vec3d calc_dr_entropy3();
	Vec3d calc_dr_entropy4();
	Vec3d calc_dr_lsd2();
	Vec3d calc_dr_lsd3();
	Vec3d calc_dr_lsd4();
	Vec3d calc_dr_epi2();
	double calc_dl_entropy2();
	Vec3d calc_t_entropy();
	Vec3d calc_epi_point_entropy2();
	Vec3d calc_dt_entropy5();
	Vec3d calc_dt_lsd2();
	Vec3d calc_dt_lsd3();
	Vec3d calc_dt_lsd4();
	Vec3d calc_e_epi2();
	void calc_du_of1();
	void calc_du_of2();
	bool calc_du_of3();
	bool calc_e_dr_of3(bool only_dr = false);
	bool calc_t_of3();
	void update_depth_of3();
	void unproject_points_of3();
	void calc_du_gof1();
	void smooth_of_of1();
	void smooth_of_of2();
	void wipe_depth(const Vec3d& t);
	void create_keyframe(Image* image);
	void update_depth();
	void update_depth_lsd2();
	void smooth_depth_lsd2();

	void build_of3(BuildFlag flag);
	void build_of4(BuildFlag flag);
	void prepare_du_of4();
	bool calc_du_of4();
	bool calc_e_dr_of4();

	void build_ofd(BuildFlag flag);
	void prepare_du_ofd();
	bool calc_du_ofd();
	void prepare_dr_ofd();
	bool calc_dr_ofd();

	void build_lsd5(BuildFlag flag);
	void prepare_dr_lsd5();
	bool calc_dr_lsd5();	
	void update_depth_lsd5();
	void unproject_points_lsd5();	

	void build_of5(BuildFlag flag);
	void prepare_du_of5();
	bool calc_du_of5();	
	void prepare_dr_of5();
	bool calc_dr_of5();		
	void update_depth_of5();
	//void unproject_points_lsd5();			

	void build_of6(BuildFlag flag);
	void prepare_du_of6();
	bool calc_du_of6();	
	void prepare_dr_of6();
	bool calc_dr_of6();		
	void update_depth_of6();	

	void build_lsd6(BuildFlag flag);
	void prepare_dr_lsd6();
	bool calc_dr_lsd6();
	void transfer_depth_lsd6();

	void build_of7(BuildFlag flag);
	void prepare_du_of7();
	bool calc_du_of7();	
	void prepare_dr_of7();
	bool calc_dr_of7();
	//void update_depth_of6();

	void build_lsd7(BuildFlag flag);
	void prepare_dr_lsd7();
	bool calc_dr_lsd7();
	void calc_dd_lsd7();
	void update_depth_lsd7();
	void transfer_depth_lsd7();

	void build_lsd8(BuildFlag flag);
	void prepare_dr_lsd8();
	bool calc_dr_lsd8();
	void update_depth_lsd8();
	void smooth_depth_lsd8();

	void build_lsd9(BuildFlag flag);
	void prepare_dr_lsd9();
	bool calc_dr_lsd9();
	bool update_depth_lsd9();
	bool smooth_depth_lsd9();


	VideoSource* m_source;
	bool m_working;
	Camera* m_cameras[MAX_STATIC_CAMERA_COUNT];
	int m_camera_count;
	std::vector<Camera*> m_keyframes;
	Camera* m_key;
	Camera* m_frame;
	Image* m_mask;
	//Image* m_points;
	Image* m_residual;
	Image* m_gradient;
	Image* m_depth;
	Image* m_iuux;
	Image* m_debug_image;
	Image* m_debug_weight;
	Image* m_epi_line;
	Image* m_warp;
	Image* m_weight;
	Image* m_of;
	Image* m_grad_grad[4];	// {guu guv; gvu gvv }
	Image* m_grad_residual;
	Image* m_dut;
	//Image* m_depth_weight;
	bool m_changed;
	char m_pixel_info[MAX_PIXEL_INFO];
	int m_width;
	int m_height;
	std::string m_image_name;
	double m_interval;
	double m_res[2];

};


} // namespace

#endif


/*


	
	virtual void tick();
	
	
*/
