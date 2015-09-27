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
typedef RbaEngine<
	kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean3D,          // Parameterization of landmark positions    
	observations::Cartesian_3D       // Type of observations
	> 
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with https://github.com/jlblancoc/recursive-world-toolkit 
//  and the script: tutorials_dataset-cartesian.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-4;
struct basic_euclidean_dataset_entry_t 
{
	unsigned int landmark_id;
	double x,y,z;
};
// Observations for KF#0. Ground truth pose: (xyz,q)=0.000000 0.000000 0.000000 0.996195 0.000000 0.000000 0.087156
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble(0.995247,0.003802,-0.043453,0.087073));
basic_euclidean_dataset_entry_t  observations_0[] = {
 {    98,   0.44179076,  -0.90376452,   1.21831585},
 {   124,   0.36887709,   1.51873558,   0.27846626},
 {    61,  -0.13650907,  -1.42614129,   0.84324147},
 {    67,   0.60411947,   1.58983767,   0.26060155},
 {   143,   0.65546905,  -0.28030347,   1.78930357},
 {   146,   1.85201587,   0.02629437,  -0.66017960},
 {    21,  -0.01560438,  -0.72945905,  -1.96266939},
 {    53,   0.75744877,  -1.97328401,   0.00392723},
 {   144,   1.86654080,   0.69846627,  -1.23948292},
 {   119,   0.86231273,  -0.88707413,   2.14763103},
 {   118,   1.64881194,   1.71511394,  -0.75028619},
 {    55,   1.87871121,  -1.47282934,  -0.98887906},
 {    64,   1.07097295,  -2.30504332,  -0.70275791},
 {    39,   2.62272811,  -0.36239644,  -0.48996278},
 {   107,   0.98453078,   2.52728400,   0.54441222},
 {    35,   2.81663034,  -0.04707533,  -0.64743974},
 {   114,   1.28246133,   1.67306049,   2.05237042},
 {    20,   1.34811775,   2.56802258,  -0.51725748},
 {    58,  -0.13924954,  -2.72185970,  -1.16368338},
 {    63,   0.94986696,   1.81317869,   2.17580006},
 };
// Observations for KF#10:(xyz,q)= 1.230806 0.294472 0.000000 0.999999 0.000000 0.000000 0.001357
const mrpt::poses::CPose3DQuat GT_pose10(1.226071, 0.293637, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));
basic_euclidean_dataset_entry_t  observations_10[] = {
 {   146,   0.58951282,   0.06112746,  -0.65858034},
 {    39,   1.41510972,  -0.19256191,  -0.49100010},
 {   144,   0.48965699,   0.73434404,  -1.22816493},
 {    67,  -0.90853948,   1.37461702,   0.27968160},
 {    35,   1.55245056,   0.15354730,  -0.64321834},
 {   124,  -1.12818445,   1.26409439,   0.29563350},
 {    98,  -0.64338648,  -1.12385784,   1.20105907},
 {   118,   0.10067270,   1.69149623,  -0.72533027},
 {   143,  -0.54019814,  -0.48152237,   1.78170215},
 {    55,   0.87255815,  -1.40636435,  -1.00855321},
 {    28,   1.88660810,  -0.32479903,  -0.78342142},
 {    53,  -0.14792658,  -2.10568124,  -0.02727065},
 {    61,  -1.12336633,  -1.73181994,   0.81636386},
 {    76,   2.16909436,  -0.51355650,   0.02087719},
 {    84,   1.37172698,   1.85548870,   0.16019421},
 {   141,   1.72910762,  -1.03905231,  -1.19569905},
 {   119,  -0.23312637,  -1.04921385,   2.13211166},
 {    21,  -1.11968655,  -0.98349037,  -1.97883008},
 {    64,   0.21862795,  -2.36852366,  -0.73742358},
 {    71,   1.85722805,   0.23574278,   1.64624521},
 {   107,  -0.69438136,   2.35901095,   0.57829804},
 {    20,  -0.34172474,   2.47692692,  -0.48129000},
 {    52,   0.39832690,   2.34432761,   1.03254852},
 {   114,  -0.25675973,   1.54618039,   2.07502519},
 {    37,   2.25845642,  -1.12777769,   0.83780026},
 {    63,  -0.60857470,   1.62555069,   2.19917667},
 {    59,   2.57934607,  -1.31716776,  -0.03307248},
 {   127,   1.52496362,   1.88008739,  -1.66156675},
 };

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = true;
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
		obs_field.obs.obs_data.pt.z = observations_0[i].z + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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
	mrpt::math::CMatrixDouble MAP2( sizeof(observations_10)/sizeof(observations_10[0]), 3);

	for (size_t i=0;i<sizeof(observations_10)/sizeof(observations_10[0]);i++)
	{
		obs_field.obs.feat_id = observations_10[i].landmark_id;
		obs_field.obs.obs_data.pt.x = observations_10[i].x + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.y = observations_10[i].y + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.z = observations_10[i].z + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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
