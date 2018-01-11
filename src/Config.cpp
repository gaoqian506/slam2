#include "Config.h"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>

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

	int Config::num_skip_frames = 0;
	double Config::crop_percent[2] = {1, 1};
	Config::SourceType Config::source_type = Config::Video;
	std::string Config::video_file;
	std::vector<std::string> Config::image_files;


	void Config::load(const char* config_file) {

		FILE * fp;
		// char * line = NULL;
		// char* start = 0;
		// size_t len = 0;
		// ssize_t read;

		fp = fopen(config_file, "r");
		if (fp == NULL) {
			std::cout << "Faild to load config file:" << config_file << std::endl;
		}

		char buffer[1024];
		char *first, *colon, *second, *last;
		while(fgets(buffer, 1024, fp)) {
			
			std::cout << buffer;



			if (colon = strchr(buffer, ':')) {

				first = buffer;
				last = buffer + strlen(buffer);
				second = colon+1;
				colon[0] = 0;

				while(last[0] == ' ' || last[0] == '\t' || last[0] == '\n' || last[0] == 0) {
					last[0] = 0;
					last--;
				}
				while(colon[0] == ' ' || colon[0] == '\t' || colon[0] == '\n' || colon[0] == 0) {
					colon[0] = 0;
					colon--;
				}

				while(first[0] == ' ' || first[0] == '\t' ) {
					first++;
				}
				while(second[0] == ' ' || second[0] == '\t' ) {
					second++;
				}


				if (first[0] == '#') { continue; }

				if (strcmp(first, "skips") == 0) {
					Config::num_skip_frames = atoi(second);
				}
				else if (strcmp(first, "crop_h") == 0) {
					Config::crop_percent[0] = atof(second);
				}
				else if (strcmp(first, "crop_v") == 0) {
					Config::crop_percent[1] = atof(second);
				}
				else if (strcmp(first, "video") == 0) {
					Config::source_type = Video;
					Config::video_file = second;
				}	
				else if (strcmp(first, "max_width") == 0) {
					Config::max_width = atoi(second);
				}


			}

		}

		// while ((read = getline(&line, &len, fp)) != -1) {

		// 	//for (int i = 0; i < )



		// //     printf("Retrieved line of length %zu :\n", read);
		// //     printf("%s", line);
		// }

		if (fp) { fclose(fp); }
		//if (line) { free(line); }

	}
}
