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
	typedef options::sensor_pose_on_robot_se3       sensor_pose_on_robot_t;
	typedef options::observation_noise_identity     obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)

	// Solver algorithm:
	typedef options::solver_LM_schur_dense_cholesky      solver_t;
	//typedef options::solver_LM_schur_sparse_cholesky     solver_t;
	//typedef options::solver_LM_no_schur_sparse_cholesky  solver_t;

};

typedef RbaEngine<
	kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
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
// Observations for KF#0: 0.000000 0.000000 0.000000 0.564787 -0.517532 0.434261 -0.473913
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble(0.564787, -0.517532, 0.434261, -0.473913));
basic_stereo_dataset_entry_t  dataset0[] = {
 {   146,  509.16046143,  437.46981812,  487.56234741,  437.46981812},
 {   144,  437.15927124,  483.60800171,  415.72924805,  483.60800171},
 {   119,  717.74304199,   10.41780090,  671.35614014,   10.41780090},
 {   118,  303.95758057,  452.25698853,  279.69769287,  452.25698853},
 {    55,  668.79144287,  462.95404053,  647.50024414,  462.95404053},
 {    64,  942.45770264,  482.42794800,  905.10852051,  482.42794800},
 {    39,  539.63507080,  412.02212524,  524.38378906,  412.02212524},
 {    35,  515.34265137,  418.47946167,  501.14129639,  418.47946167},
 {   114,  251.08596802,  143.94938660,  219.89593506,  143.94938660},
 {    20,  131.02098083,  441.55328369,  101.34998322,  441.55328369},
 {    63,  130.22460938,   40.40441895,   88.11344147,   40.40441895},
 {    28,  549.69879150,  422.24081421,  536.64208984,  422.24081421},
 {    52,  285.58596802,  311.38674927,  266.17193604,  311.38674927},
 {   141,  602.49987793,  447.69750977,  588.13580322,  447.69750977},
 {    84,  397.96939087,  377.29605103,  384.32940674,  377.29605103},
 {    76,  560.31323242,  382.84091187,  548.23840332,  382.84091187},
 {    71,  511.07693481,  305.64028931,  498.33679199,  305.64028931},
 {    95,  489.02133179,  574.60296631,  471.31988525,  574.60296631},
 {    37,  597.42114258,  345.28842163,  585.29571533,  345.28842163},
 {    18,  872.09954834,  508.56875610,  848.17993164,  508.56875610},
 {   100,  274.51089478,  287.06481934,  257.16955566,  287.06481934},
 {   127,  405.27337646,  466.33535767,  392.28976440,  466.33535767},
 {   115,  298.95901489,  357.22961426,  283.78897095,  357.22961426},
 {    59,  604.91369629,  384.71456909,  593.73840332,  384.71456909},
 {     6,  604.14477539,  436.54782104,  593.01409912,  436.54782104},
 {   113,  410.54818726,  411.51428223,  399.81372070,  411.51428223},
 {    99,  321.10958862,  432.39556885,  307.92941284,  432.39556885},
 {   128,  452.99237061,  360.34350586,  443.32141113,  360.34350586},
 {    86,  482.48013306,  347.64862061,  473.00042725,  347.64862061},
 {    30,  995.84106445,  405.77438354,  973.26440430,  405.77438354},
 {    13,  485.14672852,  439.37075806,  475.89678955,  439.37075806},
 {   148,  880.86297607,  206.19326782,  860.58264160,  206.19326782},
};
// Observations for KF#10. 1.226071 0.293637 0.110099 0.521317 -0.478835 0.477946 -0.520108
const mrpt::poses::CPose3DQuat GT_pose10(1.226071, 0.293637, 0.110099, mrpt::math::CQuaternionDouble(0.521317, -0.478835, 0.477946, -0.520108));
basic_stereo_dataset_entry_t  dataset1[] = {
 {   146,  491.26168823,  551.57379150,  423.40914917,  551.57379150},
 {    39,  539.21508789,  436.04541016,  510.94876099,  436.04541016},
 {   144,  212.05824280,  760.23156738,  130.36854553,  760.23156738},
 {    35,  492.21871948,  446.14865112,  466.45300293,  446.14865112},
 {    55,  834.35400391,  557.37854004,  788.51184082,  557.37854004},
 {    28,  546.43200684,  446.28805542,  525.22998047,  446.28805542},
 {    76,  559.35211182,  382.55627441,  540.91125488,  382.55627441},
 {    84,  241.46690369,  366.48257446,  212.30659485,  366.48257446},
 {   141,  632.18359375,  487.72677612,  609.05029297,  487.72677612},
 {    71,  486.61346436,  251.04017639,  465.07601929,  251.04017639},
 {    37,  611.87152100,  328.35580444,  594.16027832,  328.35580444},
 {    59,  614.13189697,  385.92330933,  598.62408447,  385.92330933},
 {   127,  265.42541504,  547.43658447,  239.19529724,  547.43658447},
 {   113,  302.50097656,  429.91296387,  283.61132813,  429.91296387},
 {   128,  389.86721802,  345.30511475,  374.68167114,  345.30511475},
 {     6,  611.41735840,  457.71142578,  595.99652100,  457.71142578},
 {    86,  438.98941040,  328.56036377,  424.79483032,  328.56036377},
 {    13,  440.80871582,  464.69479370,  427.16564941,  464.69479370},
 {    26,  546.39410400,  371.82641602,  535.95281982,  371.82641602},
 {    77,  487.84552002,  382.68484497,  477.85519409,  382.68484497},
 {    66,  200.29261780,  398.92413330,  182.15275574,  398.92413330},
 {    97,  390.99511719,  399.05416870,  379.56640625,  399.05416870},
 {    32,  558.65643311,  375.37698364,  548.65789795,  375.37698364},
 {     3,  568.27807617,  334.56246948,  557.75860596,  334.56246948},
 {    94,  216.95486450,  207.21980286,  196.44139099,  207.21980286},
 {   105,  422.90161133,  424.59942627,  412.17526245,  424.59942627},
 {    24,  375.15139771,  291.60083008,  362.73464966,  291.60083008},
 {     1,  705.76177979,  346.89096069,  692.97778320,  346.89096069},
 {    12,  422.85137939,  393.76672363,  413.06048584,  393.76672363},
 {   132,  407.56420898,  505.53576660,  395.41671753,  505.53576660},
 {    74,  195.35592651,  543.41839600,  176.65670776,  543.41839600},
 {   106,  735.83538818,  419.90661621,  723.00628662,  419.90661621},
 {    81,  627.14727783,  411.86691284,  617.46148682,  411.86691284},
 {     2,  425.69473267,  355.24987793,  416.55752563,  355.24987793},
 {    85,  368.50869751,  435.69857788,  358.13888550,  435.69857788},
 {   121,  690.39721680,  402.24075317,  679.54412842,  402.24075317},
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
