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

using namespace mrpt::srba;
using namespace std;
using mrpt::utils::DEG2RAD;

// --------------------------------------------------------------------------------
// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
// --------------------------------------------------------------------------------
struct my_srba_options
{
	typedef options::sensor_pose_on_robot_se3      sensor_pose_on_robot_t;
	typedef options::observation_noise_identity    obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
	typedef options::solver_LM_schur_dense_cholesky  solver_t;
};

typedef RbaEngine<
	kf2kf_poses::SE2,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean3D,          // Parameterization of landmark positions
	observations::StereoCamera,      // Type of observations
	my_srba_options                  // Other parameters
	>
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset (generated with https://github.com/jlblancoc/recursive-world-toolkit )
// --------------------------------------------------------------------------------
struct basic_stereo_dataset_entry_t
{
	unsigned int landmark_id;
	double l_px_x, l_px_y, r_px_x, r_px_y;
};
// Observations for KF#0
basic_stereo_dataset_entry_t  dataset0[] = {
 {    39,  539.29449463,  384.00000000,  524.23114014,  384.00000000},
 {    35,  515.28930664,  384.00000000,  501.31469727,  384.00000000},
 {    20,  141.98646545,  384.00000000,  113.16947174,  384.00000000},
 {    28,  549.01715088,  384.00000000,  536.19653320,  384.00000000},
 {    18,  848.99078369,  384.00000000,  826.60614014,  384.00000000},
 {    37,  599.72821045,  384.00000000,  587.27532959,  384.00000000},
 {     6,  601.74615479,  384.00000000,  590.90521240,  384.00000000},
 {    13,  485.88748169,  384.00000000,  476.89270020,  384.00000000},
 {    30,  991.59832764,  384.00000000,  969.21966553,  384.00000000},
 {    24,  437.52539063,  384.00000000,  428.89263916,  384.00000000},
 {     3,  580.46923828,  384.00000000,  571.96411133,  384.00000000},
 {    26,  562.13531494,  384.00000000,  553.93872070,  384.00000000},
 {     8,  681.00463867,  384.00000000,  670.68438721,  384.00000000},
 {     1,  695.37768555,  384.00000000,  684.69763184,  384.00000000},
 {    32,  572.87854004,  384.00000000,  564.88854980,  384.00000000},
 {    11,  721.99755859,  384.00000000,  711.39343262,  384.00000000},
 {    12,  470.31097412,  384.00000000,  463.17254639,  384.00000000},
 {     5,  554.08819580,  384.00000000,  547.04064941,  384.00000000},
 {     2,  470.66183472,  384.00000000,  463.77709961,  384.00000000},
 {    36,  941.68115234,  384.00000000,  925.83496094,  384.00000000},
 {    22,  712.12817383,  384.00000000,  702.74102783,  384.00000000},
 {    46,  660.98260498,  384.00000000,  653.19744873,  384.00000000},
 {    47,  412.31518555,  384.00000000,  405.41717529,  384.00000000},
 {     7,  476.52420044,  384.00000000,  470.44110107,  384.00000000},
 {    17,  592.48101807,  384.00000000,  586.03741455,  384.00000000},
 {    42,  538.14855957,  384.00000000,  532.17541504,  384.00000000},
 {    27,  705.59130859,  384.00000000,  697.47851563,  384.00000000},
 {    31,  374.95474243,  384.00000000,  367.96514893,  384.00000000},
};

// Observations for KF#1. GT pose: 10 -> 0.502744 -0.099901 0.000000 0.526224 -0.526224 0.472322 -0.472322
basic_stereo_dataset_entry_t  dataset1[] = {
 {    39,  538.84362793,  384.00000000,  511.19470215,  384.00000000},
 {    35,  492.97552490,  384.00000000,  467.92294312,  384.00000000},
 {    28,  545.65521240,  384.00000000,  525.04907227,  384.00000000},
 {    37,  615.96533203,  384.00000000,  597.56689453,  384.00000000},
 {     6,  608.14080811,  384.00000000,  593.25360107,  384.00000000},
 {    13,  443.73529053,  384.00000000,  430.62341309,  384.00000000},
 {    24,  367.12982178,  384.00000000,  353.96624756,  384.00000000},
 {     3,  570.37628174,  384.00000000,  559.49926758,  384.00000000},
 {    26,  546.97644043,  384.00000000,  536.41058350,  384.00000000},
 {    32,  559.28576660,  384.00000000,  549.18865967,  384.00000000},
 {     8,  693.45947266,  384.00000000,  680.74969482,  384.00000000},
 {     1,  711.24810791,  384.00000000,  698.11474609,  384.00000000},
 {    12,  423.10546875,  384.00000000,  413.32467651,  384.00000000},
 {     5,  533.79895020,  384.00000000,  525.03625488,  384.00000000},
 {     2,  424.05053711,  384.00000000,  414.72103882,  384.00000000},
 {    11,  736.43890381,  384.00000000,  723.78930664,  384.00000000},
 {    22,  715.67138672,  384.00000000,  704.84838867,  384.00000000},
 {    47,  334.04904175,  384.00000000,  324.01797485,  384.00000000},
 {     7,  433.45681763,  384.00000000,  425.53250122,  384.00000000},
 {    46,  653.96527100,  384.00000000,  645.04693604,  384.00000000},
 {    42,  512.31976318,  384.00000000,  505.06820679,  384.00000000},
 {    17,  575.66925049,  384.00000000,  568.13891602,  384.00000000},
 {    36,  988.72906494,  384.00000000,  970.40332031,  384.00000000},
 {     9,  558.54290771,  384.00000000,  551.54156494,  384.00000000},
 {    31,  268.40109253,  384.00000000,  257.69583130,  384.00000000},
 {     0,  448.58941650,  384.00000000,  441.61938477,  384.00000000},
 {    40,  608.29187012,  384.00000000,  600.95947266,  384.00000000},
 {    27,  698.85803223,  384.00000000,  689.85357666,  384.00000000},
 {    16,  550.01416016,  384.00000000,  543.33697510,  384.00000000},
 {    15,  486.11828613,  384.00000000,  479.63989258,  384.00000000},
 {    10,  479.26995850,  384.00000000,  472.76791382,  384.00000000},
 {    48,  358.30957031,  384.00000000,  350.39227295,  384.00000000},
 {    38,  390.95483398,  384.00000000,  383.80099487,  384.00000000},
 {    29,  392.46899414,  384.00000000,  385.34008789,  384.00000000},
};

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = true;
	rba.parameters.obs_noise.std_noise_observations = 0.5; // pixels

	// =========== Topology parameters ===========
	rba.parameters.srba.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.srba.max_tree_depth       = 3;
	rba.parameters.srba.max_optimize_depth   = 3;
	// ===========================================

	// Set camera calib:
	mrpt::utils::TCamera & lc = rba.parameters.sensor.camera_calib.leftCamera;
	lc.ncols = 1024;
	lc.nrows = 768;
	lc.cx(512);
	lc.cy(384);
	lc.fx(200);
	lc.fy(150);
	lc.dist.setZero();
	rba.parameters.sensor.camera_calib.rightCamera = lc;
	rba.parameters.sensor.camera_calib.rightCameraPose.fromString("[0.2 0 0  1 0 0 0]");  // [X Y Z qr qx qy qz]

	// Sensor pose on the robot parameters:
	rba.parameters.sensor_pose.relative_pose = mrpt::poses::CPose3D(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90) ); // Set camera pointing forwards (camera's +Z is robot +X)

	// Alternatively, parameters can be loaded from an .ini-like config file
	// -----------------------------------------------------------------------
	// rba.parameters.loadFromConfigFileName("config_file.cfg", "srba");
	//rba.sensor_params.camera_calib.loadFromConfigFile("CAMERA","config_file.cfg");

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
#if 1
	obs_field.is_unknown_with_init_val = false; // We don't have any guess on the initial LM position (will invoke the inverse sensor model)
#else
	obs_field.is_unknown_with_init_val = true;      // Set default relative position of unknown new landmarks...
	obs_field.setRelPos( mrpt::math::TPoint3D(0.,0.,1.) ); // ...to (x,y,z)=(0,0,1), in front of the camera.
#endif

	for (size_t i=0;i<sizeof(dataset0)/sizeof(dataset0[0]);i++)
	{
		obs_field.obs.feat_id = dataset0[i].landmark_id;
		obs_field.obs.obs_data.left_px.x = dataset0[i].l_px_x;
		obs_field.obs.obs_data.left_px.y = dataset0[i].l_px_y;
		obs_field.obs.obs_data.right_px.x = dataset0[i].r_px_x;
		obs_field.obs.obs_data.right_px.y = dataset0[i].r_px_y;
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
	// Define observations of KF #1:
	// --------------------------------------------------------------------------------
	list_obs.clear();

	for (size_t i=0;i<sizeof(dataset1)/sizeof(dataset1[0]);i++)
	{
		obs_field.obs.feat_id = dataset1[i].landmark_id;
		obs_field.obs.obs_data.left_px.x = dataset1[i].l_px_x;
		obs_field.obs.obs_data.left_px.y = dataset1[i].l_px_y;
		obs_field.obs.obs_data.right_px.x = dataset1[i].r_px_x;
		obs_field.obs.obs_data.right_px.y = dataset1[i].r_px_y;
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
