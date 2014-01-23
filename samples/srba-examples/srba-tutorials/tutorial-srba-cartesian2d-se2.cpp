/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/srba.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene
#include <mrpt/random.h>

using namespace mrpt::srba;
using namespace mrpt::random;
using namespace std;

// --------------------------------------------------------------------------------
// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
// --------------------------------------------------------------------------------
typedef RbaEngine<
	kf2kf_poses::SE2,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean2D,          // Parameterization of landmark positions
	observations::Cartesian_2D       // Type of observations
	>
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/
//  and the script: tutorials_dataset-cartesian.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-2;

struct basic_euclidean_dataset_entry_t
{
	unsigned int landmark_id;
	double x,y,z;
};
// Observations for KF#0
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble( 0.996195,0.,0.,0.087156));

basic_euclidean_dataset_entry_t  observations_0[] = {
 {    21,   0.15551290,  -0.72945905,   0.00000000},
 {    23,   0.43314868,  -1.98272542,   0.00000000},
 {    39,   2.65545091,  -0.36239644,   0.00000000},
 {    35,   2.86234031,  -0.04707532,   0.00000000},
 {    20,   1.38806971,   2.56802258,   0.00000000},
 {    28,   3.11997887,  -0.57746373,   0.00000000},
 {    18,   1.78694047,  -3.01091272,   0.00000000},
 {    37,   3.21209594,  -1.40895775,   0.00000000},
 {    45,   1.24498942,  -3.32446676,   0.00000000},
 {     6,   3.68972255,  -1.65569177,   0.00000000},
 {    13,   4.44701777,   0.58061391,   0.00000000},
 {    43,   1.00330919,  -4.40925967,   0.00000000},
 {    30,   1.78741597,  -4.28620926,   0.00000000},
 {    24,   4.63351881,   1.72539759,   0.00000000},
 {     3,   4.70305332,  -1.61007267,   0.00000000},
 };
// Observations for KF#10:
const mrpt::poses::CPose3DQuat GT_pose10(1.230806,0.294472,0, mrpt::math::CQuaternionDouble(  0.999999,0.,0.,0.001357));

basic_euclidean_dataset_entry_t  observations_10[] = {
 {    21,  -0.95365746,  -0.98326083,   0.00000000},
 {    39,   1.44671094,  -0.19417495,   0.00000000},
 {    35,   1.59664177,   0.15187617,   0.00000000},
 {    28,   1.94117010,  -0.32665253,   0.00000000},
 {    23,  -0.46583194,  -2.17060428,   0.00000000},
 {    37,   2.17409589,  -1.13015327,   0.00000000},
 {    20,  -0.30303829,   2.47640333,   0.00000000},
 {     6,   2.68687505,  -1.29159175,   0.00000000},
 {    18,   1.04382049,  -2.95218658,   0.00000000},
 {    13,   3.05066582,   1.04126389,   0.00000000},
 {    45,   0.56345958,  -3.35378398,   0.00000000},
 {    24,   3.03869071,   2.20107806,   0.00000000},
 {     3,   3.67748505,  -1.07338963,   0.00000000},
 {    26,   3.78577874,  -0.66206601,   0.00000000},
 {    32,   3.96151904,  -0.93661778,   0.00000000},
 {     8,   3.14718105,  -2.85542922,   0.00000000},
 {     1,   3.04568235,  -3.03423259,   0.00000000},
 {    30,   1.26233346,  -4.20862344,   0.00000000},
 {    12,   4.08965501,   1.81773981,   0.00000000},
 {    43,   0.51081104,  -4.46392502,   0.00000000},
 {     5,   4.56480438,  -0.49754058,   0.00000000},
 {     2,   4.28747355,   1.88540463,   0.00000000},
 {    11,   3.16214953,  -3.54854757,   0.00000000},
 };

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel= true;
	rba.parameters.obs_noise.std_noise_observations = 0.1;

	// =========== Topology parameters ===========
	rba.parameters.srba.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.srba.max_tree_depth       = 3;
	rba.parameters.srba.max_optimize_depth   = 3;
	// ===========================================

	// Set sensors parameters:
	// rba.sensor_params has no parameters for the "Cartesian" sensor.

	// Alternatively, parameters can be loaded from an .ini-like config file
	// -----------------------------------------------------------------------
	// rba.parameters.loadFromConfigFileName("config_file.cfg", "srba");

	// --------------------------------------------------------------------------------
	// Dump parameters to console (for checking/debugging only)
	// --------------------------------------------------------------------------------
	//cout << "RBA parameters:\n-----------------\n";
	//rba.parameters.dumpToConsole();

	// --------------------------------------------------------------------------------
	// Define observations of KF #0:
	// --------------------------------------------------------------------------------
	my_srba_t::new_kf_observations_t  list_obs;
	my_srba_t::new_kf_observation_t   obs_field;

	obs_field.is_fixed = false;   // Landmarks have unknown relative positions (i.e. treat them as unknowns to be estimated)
	obs_field.is_unknown_with_init_val = false; // We don't have any guess on the initial LM position (will invoke the inverse sensor model)

	for (size_t i=0;i<sizeof(observations_0)/sizeof(observations_0[0]);i++)
	{
		obs_field.obs.feat_id = observations_0[i].landmark_id;
		obs_field.obs.obs_data.pt.x = observations_0[i].x + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.y = observations_0[i].y + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		list_obs.push_back( obs_field );
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


	// --------------------------------------------------------------------------------
	// Define observations of next KF:
	// --------------------------------------------------------------------------------
	list_obs.clear();

	for (size_t i=0;i<sizeof(observations_10)/sizeof(observations_10[0]);i++)
	{
		obs_field.obs.feat_id = observations_10[i].landmark_id;
		obs_field.obs.obs_data.pt.x = observations_10[i].x + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.y = observations_10[i].y + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		list_obs.push_back( obs_field );
	}

	//  Here happens the main stuff: create Key-frames, build structures, run optimization, etc.
	//  ============================================================================================
	rba.define_new_keyframe(
		list_obs,      // Input observations for the new KF
		new_kf_info,   // Output info
		true           // Also run local optimization?
		);

	cout << "Created KF #" << new_kf_info.kf_id
		<< " | # kf-to-kf edges created:" <<  new_kf_info.created_edge_ids.size() << endl
		<< "Optimization error: " << new_kf_info.optimize_results.total_sqr_error_init << " -> " << new_kf_info.optimize_results.total_sqr_error_final << endl
		<< "-------------------------------------------------------" << endl;


	// Dump the relative pose of KF#0 wrt KF#1:
	cout << "inv_pose of KF-to-KF edge #0 (relative pose of KF#0 wrt KF#1):\n" << rba.get_k2k_edges()[0].inv_pose << endl;
	cout << "Relative pose of KF#1 wrt KF#0:\n" << (-rba.get_k2k_edges()[0].inv_pose) << endl;

	// Compare to ground truth:
	cout << "Ground truth: relative pose of KF#1 wrt KF#0: \n" << mrpt::poses::CPose3D(GT_pose10-GT_pose0) << endl;

	// --------------------------------------------------------------------------------
	// Saving RBA graph as a DOT file:
	// --------------------------------------------------------------------------------
	const string sFil = "graph.dot";
	cout << "Saving final graph of KFs and LMs to: " << sFil << endl;
	rba.save_graph_as_dot(sFil, true /* LMs=save */);
	cout << "Done.\n";

	// --------------------------------------------------------------------------------
	// Show 3D view of the resulting map:
	// --------------------------------------------------------------------------------
	my_srba_t::TOpenGLRepresentationOptions  opengl_options;
	mrpt::opengl::CSetOfObjectsPtr rba_3d = mrpt::opengl::CSetOfObjects::Create();

	rba.build_opengl_representation(
		0,  // Root KF,
		opengl_options, // Rendering options
		rba_3d  // Output scene
		);

	// Display:
#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D win("RBA results",640,480);
	{
		mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
		scene->insert(rba_3d);
		win.unlockAccess3DScene();
	}
	win.setCameraZoom( 4 );
	win.repaint();

	cout << "Press any key or close window to exit.\n";
	win.waitForKey();
#endif

	return 0; // All ok
}
