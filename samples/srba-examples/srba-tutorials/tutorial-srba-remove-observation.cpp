/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

//#define SRBA_DETAILED_TIME_PROFILING   1

#include <mrpt/srba.h>
#include <mrpt/random.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene

using namespace mrpt::srba;
using namespace std;
using mrpt::utils::DEG2RAD;
using mrpt::utils::square;

// --------------------------------------------------------------------------------
// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
// --------------------------------------------------------------------------------
struct my_srba_options
{
	typedef options::sensor_pose_on_robot_none           sensor_pose_on_robot_t;  // sensor pose == robot pose
	typedef options::observation_noise_constant_matrix<observations::RelativePoses_2D>   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to some given matrix
	typedef options::solver_LM_schur_dense_cholesky  solver_t;
};

typedef RbaEngine<
	kf2kf_poses::SE2,                // Parameterization  KF-to-KF poses
	landmarks::RelativePoses2D,      // Parameterization of landmark positions
	observations::RelativePoses_2D,  // Type of observations
	my_srba_options                  // Other parameters
	>
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset (generated with http://code.google.com/p/recursive-world-toolkit/ )
// --------------------------------------------------------------------------------
const double STD_NOISE_XY = 0.001;
const double STD_NOISE_YAW = DEG2RAD(0.05);

struct basic_graph_slam_dataset_entry_t
{
	unsigned int current_kf;
	unsigned int observed_kf;
	double x,y,z, yaw,pitch,roll,  qr,qx,qy,qz; // Relative pose of "observed_kf" as seen from "current_kf"
};

// Corrupt observation test: add it, later remove (e.g. when it's clear it's an outlier):
const size_t obs_index_to_remove = 2; 
const size_t kf_at_which_do_remove = 8;

basic_graph_slam_dataset_entry_t dataset[] = {
 {     1,      0,     -1.78055512,      1.11331694,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     2,      1,     -1.71545942,      2.05914961,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
//---
//Original: {     2,      0,     -2.96619160,      3.74601658,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
//Corrupt observation: 
 {     2,      0,     +8.00000000,      -17.50000000,      0.00000000,   +0.70000000,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
//---
 {     3,      2,     -1.15014065,      2.45631509,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     4,      3,     -0.71839088,      2.17858845,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     4,      2,     -0.89163376,      4.88530127,      0.00000000,     -0.74799839,     -0.00000000,      0.00000000,      0.93087372,      0.00000000,     -0.00000000,     -0.36534109},
 {     5,      4,     -1.33870852,      1.73631597,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {     5,      3,     -1.21151269,      4.02676447,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {     6,      5,     -1.67977719,      2.03565806,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     6,      4,     -2.29159821,      4.14103420,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {     7,      6,     -1.49006905,      2.30876608,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     8,      7,     -1.15992524,      2.21845386,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {     9,      8,     -1.28889269,      1.78614744,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     9,      7,     -1.55814427,      4.27501619,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {    10,      9,     -1.67026750,      1.96210498,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {    10,      8,     -2.21751078,      4.09566815,      0.00000000,     -0.74799839,     -0.00000000,      0.00000000,      0.93087372,      0.00000000,     -0.00000000,     -0.36534109},
 {    11,     10,     -1.55210516,      2.27651848,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {    12,     11,     -1.21625554,      2.27164636,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {    13,     12,     -1.45455725,      1.51179033,      0.00000000,     -0.22440012,     -0.00000000,      0.00000000,      0.99371217,      0.00000000,     -0.00000000,     -0.11196480},
 {    13,     11,     -2.13482825,      3.99712454,      0.00000000,     -0.59839931,     -0.00000000,      0.00000000,      0.95557270,      0.00000000,     -0.00000000,     -0.29475552},
 {    14,     13,     -2.36655195,      0.41536284,      0.00000000,      0.00000000,     -0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000},
 {    14,     12,     -3.82110920,      1.92715317,      0.00000000,     -0.22440012,     -0.00000000,      0.00000000,      0.99371217,      0.00000000,     -0.00000000,     -0.11196480},
 {    15,     14,     -2.74448431,     -0.11373775,      0.00000000,      0.00000000,     -0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000},
 {    15,      0,      4.16910212,      0.67638546,      0.00000000,      1.57079633,     -0.00000000,      0.00000000,      0.70710678,      0.00000000,      0.00000000,      0.70710678},
 {    16,      0,      1.58658626,      0.30349575,      0.00000000,      1.57079633,     -0.00000000,      0.00000000,      0.70710678,      0.00000000,      0.00000000,      0.70710678},
 {    16,     15,     -2.58251586,     -0.37288971,      0.00000000,      0.00000000,     -0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000},
 {    16,      1,      1.97243380,      2.36770815,      0.00000000,      1.94479552,     -0.00000000,      0.00000000,      0.56332003,      0.00000000,      0.00000000,      0.82623879},
};

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = false;
	//rba.parameters.srba.optimize_new_edges_alone  = false;  // skip optimizing new edges one by one? Relative graph-slam without landmarks should be robust enough, but just to make sure we can leave this to "true" (default)

	// Information matrix for relative pose observations:
	{
		Eigen::Matrix3d ObsL;
		ObsL.setZero();
		ObsL(0,0) = 1/square(STD_NOISE_XY); // x
		ObsL(1,1) = 1/square(STD_NOISE_XY); // y
		ObsL(2,2) = 1/square(STD_NOISE_YAW); // phi

		// Set:
		rba.parameters.obs_noise.lambda = ObsL;
	}

	// =========== Topology parameters ===========
	rba.parameters.srba.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.srba.max_tree_depth       = 3;
	rba.parameters.srba.max_optimize_depth   = 3;
	rba.parameters.srba.submap_size          = 5;
	rba.parameters.srba.min_obs_to_loop_closure = 1;
	// ===========================================

	// --------------------------------------------------------------------------------
	// Dump parameters to console (for checking/debugging only)
	// --------------------------------------------------------------------------------
	cout << "RBA parameters:\n-----------------\n";
	rba.parameters.srba.dumpToConsole();

#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D win("RBA results",640,480);
#endif

	// --------------------------------------------------------------------------------
	// Process the dataset:
	// --------------------------------------------------------------------------------
	const size_t nObs = sizeof(dataset)/sizeof(dataset[0]);
	size_t cur_kf = 0; // Start at keyframe #0 in the dataset

	for (size_t obsIdx = 0; obsIdx<nObs;  cur_kf++ /* move to next KF */  )
	{
		// Create list of observations for keyframe: "cur_kf"
		my_srba_t::new_kf_observations_t  list_obs;

		// To emulate graph-SLAM, each keyframe MUST have exactly ONE fixed "fake landmark", representing its pose:
		// ------------------------------------------------------------------------------------------------------------
		{
			my_srba_t::new_kf_observation_t obs_field;
			obs_field.is_fixed = true;
			obs_field.obs.feat_id = cur_kf; // Feature ID == keyframe ID
			obs_field.obs.obs_data.x = 0;   // Landmark values are actually ignored.
			obs_field.obs.obs_data.y = 0;
			obs_field.obs.obs_data.yaw = 0;
			list_obs.push_back( obs_field );
		}

		// The rest "observations" are real observations of relative poses:
		// -----------------------------------------------------------------
		while ( dataset[obsIdx].current_kf == cur_kf && obsIdx<nObs )
		{
			my_srba_t::new_kf_observation_t obs_field;
			obs_field.is_fixed = false;   // "Landmarks" (relative poses) have unknown relative positions (i.e. treat them as unknowns to be estimated)
			obs_field.is_unknown_with_init_val = false; // Ignored, since all observed "fake landmarks" already have an initialized value.

			obs_field.obs.feat_id      = dataset[obsIdx].observed_kf;
			obs_field.obs.obs_data.x   = dataset[obsIdx].x + mrpt::random::randomGenerator.drawGaussian1D(0,STD_NOISE_XY);
			obs_field.obs.obs_data.y   = dataset[obsIdx].y + mrpt::random::randomGenerator.drawGaussian1D(0,STD_NOISE_XY);
			obs_field.obs.obs_data.yaw = dataset[obsIdx].yaw  + mrpt::random::randomGenerator.drawGaussian1D(0,STD_NOISE_YAW);

			list_obs.push_back( obs_field );
			obsIdx++; // Next dataset entry
		}

		//  Here happens the main stuff: create Key-frames, build structures, run optimization, etc.
		//  ============================================================================================
		my_srba_t::TNewKeyFrameInfo new_kf_info;
		rba.define_new_keyframe(
			list_obs,      // Input observations for the new KF
			new_kf_info,   // Output info
			true           // Also run local optimization?
			);

		cout << "Created KF #" << new_kf_info.kf_id
			<< " | # kf-to-kf edges created:" <<  new_kf_info.created_edge_ids.size()  << endl
			<< "Optimization error: " << new_kf_info.optimize_results.total_sqr_error_init << " -> " << new_kf_info.optimize_results.total_sqr_error_final << endl
			<< "-------------------------------------------------------" << endl;

		// Remove observation test:
		if (cur_kf==kf_at_which_do_remove)
		{
			MRPT_TODO("XXX")
			// obs_index_to_remove = 2; 
		}


	// Display:
#if MRPT_HAS_WXWIDGETS
		// --------------------------------------------------------------------------------
		// Show 3D view of the resulting map:
		// --------------------------------------------------------------------------------
		my_srba_t::TOpenGLRepresentationOptions  opengl_options;
		mrpt::opengl::CSetOfObjectsPtr rba_3d = mrpt::opengl::CSetOfObjects::Create();

		rba.build_opengl_representation(
			new_kf_info.kf_id ,  // Root KF: the current (latest) KF
			opengl_options, // Rendering options
			rba_3d  // Output scene
			);

		{
			mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
			scene->clear();
			scene->insert(rba_3d);
			win.unlockAccess3DScene();
		}
		win.repaint();

		cout << "Press any key to continue.\n";
		win.waitForKey();
#endif

	} // end-for each dataset entry


	// --------------------------------------------------------------------------------
	// Saving RBA graph as a DOT file:
	// --------------------------------------------------------------------------------
	const string sFil = "graph.dot";
	cout << "Saving final graph of KFs and LMs to: " << sFil << endl;
	rba.save_graph_as_dot(sFil, true /* LMs=save */);
	cout << "Done.\n";


	return 0; // All ok
}
