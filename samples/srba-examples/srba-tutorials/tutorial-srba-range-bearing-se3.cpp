/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
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
struct my_srba_options
{
	typedef options::sensor_pose_on_robot_none       sensor_pose_on_robot_t;  // The sensor pose coincides with the robot pose
	//typedef options::observation_noise_identity    obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
	typedef options::observation_noise_constant_matrix<observations::RangeBearing_3D>  obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and arbitrary

	typedef options::solver_LM_schur_dense_cholesky  solver_t;
};

typedef RbaEngine<
	kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean3D,          // Parameterization of landmark positions    
	observations::RangeBearing_3D,   // Type of observations
	my_srba_options                  // Other options
	>
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-range-bearing-3d.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-4;
struct basic_range_bearing_dataset_entry_t 
{
	unsigned int landmark_id;
	double range,yaw,pitch;
};

// Observations for KF#0. Ground truth pose: (xyz,q)= 0.000000 0.000000 0.000000 0.995247 0.003802 -0.043453 0.087073
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble(0.995247,0.003802,-0.043453,0.087073));

basic_range_bearing_dataset_entry_t   observations_0[] = {
 {    98,   1.57995661,  -1.11612141,  -0.88057821},
 {   124,   1.58750481,   1.33252588,  -0.17632351},
 {    61,   1.66239883,  -1.66622477,  -0.53198357},
 {    67,   1.72059795,   1.20765968,  -0.15204502},
 {   143,   1.92608851,  -0.40410293,  -1.19165567},
 {   146,   1.96633957,   0.01419675,   0.34239113},
 {    21,   2.09390189,  -1.59218478,   1.21487641},
 {    53,   2.11366834,  -1.20428778,  -0.00185802},
 {   144,   2.34694432,   0.35807227,   0.55639248},
 {   119,   2.47846784,  -0.79955155,  -1.04817877},
 {   118,   2.49461941,   0.80510532,   0.30549132},
 {    55,   2.58392412,  -0.66488196,   0.39272175},
 {    64,   2.63705829,  -1.13584981,   0.26975274},
 {    39,   2.69260050,  -0.13730595,   0.18298585},
 {   107,   2.76637849,   1.19932152,  -0.19808894},
 {    35,   2.89046723,  -0.01671179,   0.22590799},
 {   114,   2.94211876,   0.91679581,  -0.77201784},
 {    20,   2.94613590,   1.08737805,   0.17648625},
 {    58,   2.96345569,  -1.62191147,   0.40354151},
 {    63,   2.98730148,   1.08823686,  -0.81591035},
 {    23,   3.09687711,  -1.26068054,  -0.83347972},
 {    28,   3.21386016,  -0.18630799,   0.24547471},
 {    52,   3.26808549,   0.84726362,  -0.31014202},
 {   141,   3.27732824,  -0.42493052,   0.36914937},
 {    84,   3.37825912,   0.51818394,  -0.03880613},
 {    76,   3.40806843,  -0.23702528,  -0.00751110},
 {    71,   3.54231093,   0.00461531,  -0.48140070},
 {    45,   3.55010435,  -1.21450567,   0.04014509},
 {    68,   3.57398713,  -1.42712728,   0.68378257},
 {    95,   3.66312088,   0.11439174,   0.90085832},
 {    37,   3.68678484,  -0.40365290,  -0.23302471},
 {    18,   3.71358208,  -1.06381526,   0.38328004},
 {    91,   3.75799153,  -1.65835524,  -0.67656730},
 {   100,   3.87873841,   0.87088081,  -0.39445635},
 {   127,   3.87992671,   0.49019065,   0.45098085},
 {   115,   3.88108895,   0.81696074,  -0.12154930},
 {    59,   3.94675883,  -0.43490279,   0.00432028},
 };
// Observations for KF#10:(xyz,q)= 1.226070 0.293637 0.110099 0.999103 0.000160 -0.042322 0.001049
const mrpt::poses::CPose3DQuat GT_pose10(1.226071, 0.293637, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));
basic_range_bearing_dataset_entry_t  observations_10[] = {
 {   146,   0.88599661,   0.10332223,   0.83802213},
 {    39,   1.51019758,  -0.13524493,   0.33114189},
 {   144,   1.51242000,   0.98270483,   0.94766117},
 {    35,   1.68742684,   0.09858576,   0.39107551},
 {   118,   1.84320333,   1.51134950,   0.40445321},
 {    55,   1.93814292,  -1.01548620,   0.54728525},
 {    28,   2.06846167,  -0.17048903,   0.38844089},
 {    53,   2.11104700,  -1.64093228,   0.01291843},
 {    76,   2.22915825,  -0.23247995,  -0.00936564},
 {    84,   2.31303597,   0.93419094,  -0.06931260},
 {   141,   2.34502432,  -0.54109440,   0.53505424},
 {    64,   2.49028032,  -1.47875157,   0.30062864},
 {    71,   2.49298897,   0.12625740,  -0.72128471},
 {    20,   2.54628805,   1.70789406,   0.19016031},
 {    52,   2.59242989,   1.40249273,  -0.40965589},
 {   114,   2.60044783,   1.73535532,  -0.92388492},
 {    37,   2.65977766,  -0.46313367,  -0.32044489},
 {    59,   2.89638583,  -0.47213886,   0.01141878},
 {   127,   2.93616190,   0.88931571,   0.60152145},
 {    95,   3.04488843,   0.39090980,   1.22500067},
 {   115,   3.09422585,   1.27633221,  -0.16714667},
 {   113,   3.13437020,   0.80859085,   0.20829286},
 {    23,   3.14474672,  -1.69442056,  -0.80209768},
 {   128,   3.16030734,   0.54822387,  -0.21670432},
 {     6,   3.16473119,  -0.46131456,   0.41454190},
 {    86,   3.17553418,   0.35002168,  -0.33416683},
 {   100,   3.22501637,   1.36825755,  -0.49516815},
 {    45,   3.40382071,  -1.40643899,   0.05618251},
 {    18,   3.40978425,  -1.26468864,   0.43312935},
 {    99,   3.47156965,   1.18299202,   0.27180899},
 {    13,   3.48896168,   0.34197139,   0.46908459},
 {    68,   3.71363308,  -1.69773527,   0.66827742},
 {    26,   3.89957951,  -0.17030487,  -0.07981311},
};


int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = false; // true

//	rba.parameters.obs_noise.std_noise_observations = 0.03; //SENSOR_NOISE_STD;
	
	rba.parameters.obs_noise.lambda.setZero();
	rba.parameters.obs_noise.lambda(0,0) = 100;  // range
	rba.parameters.obs_noise.lambda(1,1) = 50;   // yaw 
	rba.parameters.obs_noise.lambda(2,2) = 30;   // pitch

	// =========== Topology parameters ===========
	rba.parameters.srba.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.srba.max_tree_depth       = 3;
	rba.parameters.srba.max_optimize_depth   = 3;
	//rba.parameters.srba.cov_recovery = crpNone;
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
		obs_field.obs.obs_data.range = observations_0[i].range + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.yaw = observations_0[i].yaw + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pitch = observations_0[i].pitch + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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
		obs_field.obs.obs_data.range = observations_10[i].range + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.yaw = observations_10[i].yaw + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pitch = observations_10[i].pitch + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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

