/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/srba.h>

// It seems TCLAP isn't ready to be included in different translation units, so I added this tweak...
#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <sstream>  // For stringstream
#include <memory>  // For auto_ptr, unique_ptr

// ---------------- All the parameters of this app: --------------
struct RBASLAM_Params
{
	// Declare the supported options.
	TCLAP::CmdLine cmd;
	TCLAP::ValueArg<std::string> arg_dataset;
	TCLAP::ValueArg<std::string> arg_gt_map;
	TCLAP::ValueArg<std::string> arg_gt_path;
	TCLAP::ValueArg<unsigned int> arg_max_known_feats_per_frame;
	TCLAP::SwitchArg  arg_se2,arg_se3;
	TCLAP::SwitchArg  arg_lm2d,arg_lm3d;
	TCLAP::SwitchArg  arg_graph_slam;
	TCLAP::ValueArg<std::string> arg_obs;
	TCLAP::ValueArg<std::string> arg_sensor_params;
	TCLAP::SwitchArg  arg_list_obs;
	TCLAP::SwitchArg  arg_no_gui;
	TCLAP::SwitchArg  arg_gui_step_by_step;
	TCLAP::ValueArg<std::string>  arg_profile_stats;
	TCLAP::ValueArg<unsigned int> arg_profile_stats_length;
	TCLAP::SwitchArg  arg_add_noise;
	TCLAP::ValueArg<double> arg_noise;
	TCLAP::ValueArg<double> arg_noise_ang;
	TCLAP::ValueArg<unsigned int> arg_max_tree_depth;
	TCLAP::ValueArg<unsigned int> arg_max_opt_depth;
	TCLAP::ValueArg<double> arg_max_lambda;
	TCLAP::ValueArg<unsigned int> arg_max_iters;
	TCLAP::ValueArg<std::string>       arg_edge_policy;
	TCLAP::ValueArg<unsigned int> arg_submap_size;
	TCLAP::ValueArg<unsigned int> arg_verbose;
	TCLAP::ValueArg<int> arg_random_seed;
	TCLAP::ValueArg<std::string> arg_rba_params_cfg_file;
	TCLAP::ValueArg<std::string> arg_write_rba_params_cfg_file;
	TCLAP::ValueArg<std::string> arg_video;
	TCLAP::ValueArg<unsigned int> arg_gui_delay;
	TCLAP::ValueArg<double> arg_video_fps;
	TCLAP::SwitchArg   arg_debug_dump_cur_spantree;
	TCLAP::ValueArg<std::string> arg_save_final_graph;
	TCLAP::ValueArg<std::string> arg_save_final_graph_landmarks;
	TCLAP::SwitchArg   arg_eval_overall_sqr_error;
	TCLAP::SwitchArg   arg_eval_overall_se3_error;
	TCLAP::SwitchArg   arg_eval_connectivity;

	// Parse all cmd-line arguments at construction
	// ---------------------------------------------------
	RBASLAM_Params(int argc, char**argv);

};


// ---------------- The main RBA-Runner class -----------------------
struct RBA_Run_Base
{
	virtual int run(RBASLAM_Params &params) = 0;

	RBA_Run_Base() {}
	virtual ~RBA_Run_Base() {}
};

#if MRPT_HAS_CXX11
	typedef std::unique_ptr<RBA_Run_Base> RBA_Run_BasePtr;
#else
	typedef std::auto_ptr< RBA_Run_Base > RBA_Run_BasePtr;
#endif

// Forward decl:
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE, class RBA_OPTIONS>
struct RBA_Run;

// ----------- RBA Problem factories ----------------------------
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
struct RBA_Run_Factory
{
	// Fwd. declaration: instantiated in separate .cpp files to avoid huge RAM requirements while compiling.
	static RBA_Run_BasePtr create();
};

// ----------- A registry of implemented RBA problems -------------------
// Each implementation should return an empty pointer if the params do not
// match the RBA problem type, or a problem object if it does.
typedef RBA_Run_BasePtr (*factory_functor_t)( RBASLAM_Params &params);

struct RBA_implemented_registry
{
	// It's a singleton:
	static RBA_implemented_registry & getInstance();

	void doRegister(factory_functor_t functor, const std::string &description);
	RBA_Run_BasePtr searchImplementation( RBASLAM_Params &params) const;
	void dumpAllKnownProblems() const;

private:
	struct TEntry
	{
		const factory_functor_t functor;
		const std::string       description;
		TEntry(const factory_functor_t functor_, const std::string &description_ ) : functor(functor_),description(description_)
		{ }
	};
	typedef std::list<TEntry> list_t;

	RBA_implemented_registry() { }
	list_t m_registered;
};

// Declare a "srba_options_t" type for each kind of problem:
// ------------------------------------------------------------------------
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
struct problem_options_traits_t
{
	// Default implementation:
	typedef mrpt::srba::RBA_OPTIONS_DEFAULT srba_options_t;

};


