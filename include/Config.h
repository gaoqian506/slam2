#ifndef __WW_CONFIG_HEADER__
#define __WW_CONFIG_HEADER__

#include "Vectors.h"

namespace ww {



class Config {

public:

	enum Method { 
		Lsd, 
		Epipolar, 
		Entropy, 	// E(e) = dg*dot(l, e)
					// E(r) = cross(l, e)*(Ir*dr-dg)
					// base on current frame

		Entropy2, 	// E(e) = dg*dot(l, e)
					// E(r) = cross(l, e)*(Ir*dr-dg)
					// base on key frame

		Entropy3, 	// E = (dg-Ir*dr)*dot(l, e)

		Entropy4, 	// E(e) = (dg)*dot(It, e)
					// E(dr) = (dg)*dot(Ir, dr)

		Entropy5, 	// E(dt) = (dg)*dot(It, dt)
					// E(dr) = (dg)*dot(Ir, dr)

		Lsd2,	 	// E(dt) = dot(It, dt)-dg
					// E(dr) = dot(Ir, dr)-dg

		Lsd3,	 	// E(dt) = (dot(It, dt)-dg)^2
					// E(dr) = (dot(Ir, dr)-dg)^2

		Epi2,		// e = dot(dgl, e)
					// E(dr) = w(dot(Ir, dr)-dg)^2
					// w = exp(-dot(l,e)/sigma)
		Lsd4,	 	// E(dt) = (dot(It, dt)-dg)^2
					// E(dr) = w(dot(Ir, dr)-dg)^2
					// w = dot(Ir,t);
		Of1,		// E = Iudu-dg
					// of[i] = mean(of[i]);
		Gof1,		// gradient-based optical flow
					// E = Gudu-dg (g:gradient)
		Of2,		// IuUt + It = 0;
					// Ut = sum(wUt)/W, W = dot(Iu0, Iu1)
		Of3,		// E(dut) = (iu0dut-it)^2+(iu1dut-it)^+
					//		lamda w(iu0(dut-duu)^2)
		Of4,		// E(e) = w dot(du, ux e);
					// E(dr) = w dot((x0+xr dr), cross(x1, e));
					// move mediat result to keyframe
		Ofd,		// optical flow and intensity
		Lsd5,	 	// E(dt) = w(dot(It, dt)-dg)^2
					// E(dr) = w(dot(Ir, dr)-dg)^2
					// w = dot(iu0, wiu1);
		Of5,		// E(r) = w a dr = b
					// w = p(ofr) * dot(iu0, iu1);
					// a = ur
					// b = ofr
		Of6,		// //use canonical intrinsic
					// estimate scene plane 
					// w = dot(iu0, iu1);
		Lsd6,	 	// E(dt) = (dot(It, dt)-dg)^2
					// E(dr) = (dot(Ir, dr)-dg)^2
					// E(d) = (dot(Id, dd)-dg)^2
		Of7,		// E(Rt) = (UrRt + Ut)^2
		Lsd7,	 	// E(dr) = (dot(Ir, dr)-dg)^2
					// E(ds) = (dot(Is, ds)-dg)^2 + (ds-dsi)^2
		Lsd8,	 	// E =  w1(I(D, R, t, u)t)^2 +
					//		w2(I(PI, R, t, u)t)^2;
		Lsd9,	 	// E =  w1(I(D, R, t, u)t)^2 +
					//		w2(I(PI, R, t, u)t)^2;
					// E(dpi) = (dot(Ipi, dpi)+it)^2 +
					//			wi(dpi - dpii)^2 +
					//			wpi(dpi - dpid)^2;


	};

	static int max_width;
	static bool manually_content;
	//static bool epipolar_mode;
	static Method method;
	static int of_skip;
	static int field_skip;
	static double sigma2_dgdu; //0.2*0.2;
	static double sigma2_dgdt; //0.2*0.2;
	static double du_smooth_lamda_of3;
	static double stable_factor_of3;
	static double min_weight_of3;
	static bool use_i1_constrain_of3;
	static double epi_sigma2_of3;
	static int max_iterate_times;
	
	static bool only_calc_epi_dr;
	static bool use_of_smooth_of4;
	static double min_depth_weight;

	static bool use_i1_constrain_lsd5;
	static double min_smooth_weight_lsd5;
	static double smooth_lamda_lsd5;
	static int max_iterate_times_lsd5;

	static double du_smooth_weight_of5;
	static double du_smooth_lamda_of5;
	static int max_iterations_of5;	

	static bool image_switch;
	static int win_size[2];
	//static bool use_canonical_intrinsic;

	static bool use_trace_A_lsd6;
	static int depth_grid_size_lsd6[2];
	static double default_depth_lsd6;
	static bool use_wiu1_lsd6;
	static double iu0_lenth2_thresh_lsd6;


	static bool smooth_input_image;
	static int build_steps;
	static int build_iterations;
	static double mask_radio_thresh;
	static bool use_i1_constraint;
	static double default_depth;
	static double default_ddepth;
	static Vec3d default_camera_pos;
	static double default_plane[3];
	static bool image_switch_list[128];
	static double min_gradient_length2;
	static Vec3f default_depth_plane;
	static double default_depth_weight;


};


}

#endif
