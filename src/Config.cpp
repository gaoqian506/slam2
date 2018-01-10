#include "Config.h"

namespace ww {


	int Config::max_width = 160;
	bool Config::manually_content = true;
	//bool Config::epipolar_mode = true;
	//Config::Method Config::method = Config::Entropy5;
	Config::Method Config::method = Config::Lsd9;
	//Config::Method Config::method = Config::Epi2;
	//Config::Method Config::method = Config::Of7;
	//Config::Method Config::method = Config::Gof1;
	int Config::of_skip = 5;
	int Config::field_skip = 1;
	double Config::sigma2_dgdu = 1.0/(0.2*0.2);
	double Config::sigma2_dgdt = 1.0/(0.5*0.5);
	double Config::du_smooth_lamda_of3 = 1;
	double Config::stable_factor_of3 = 0.01;
	double Config::min_weight_of3 = 0.1;
	bool Config::use_i1_constrain_of3 = true;
	double Config::epi_sigma2_of3 = 10000;
	int Config::max_iterate_times = 100;
	
	bool Config::only_calc_epi_dr = false;
	bool Config::use_of_smooth_of4 = true;
	double Config::min_depth_weight = 0;

	bool Config::use_i1_constrain_lsd5 = true;
	double Config::min_smooth_weight_lsd5 = 0.1;
	double Config::smooth_lamda_lsd5 = 1;
	int Config::max_iterate_times_lsd5 = 50;

	double Config::du_smooth_weight_of5 = 0.01;
	double Config::du_smooth_lamda_of5 = 1;
	int Config::max_iterations_of5 = 50;

	bool Config::image_switch = true;
	int Config::win_size[2] = { 800, 800 };
	//bool Config::use_canonical_intrinsic = true;

	bool Config::use_trace_A_lsd6 = false;
	int Config::depth_grid_size_lsd6[2] = { 3, 3 };
	double Config::default_depth_lsd6 = 0.5;
	bool Config::use_wiu1_lsd6 = true;
	double Config::iu0_lenth2_thresh_lsd6 = 0.1;

	bool Config::smooth_input_image = true;
	int Config::build_steps = 5;
	int Config::build_iterations = 50;
	double Config::mask_radio_thresh = 0.8;
	bool Config::use_i1_constraint = false;
	double Config::default_depth = 0.1;
	double Config::default_ddepth = 1;
	Vec3d Config::default_camera_pos = Vec3d(0, 0, 0);
	//double Config::default_plane[3] = { -1, -3.1415926*0.5, 0.15 };
	double Config::default_plane[3] = { 0, 0, 0.1 };
	bool Config::image_switch_list[128] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  };
	double Config::min_gradient_length2 = 0.005;
	Vec3f Config::default_depth_plane = Vec3f(0, 0, 0.1);
	double Config::default_depth_weight = 0.001;
}
