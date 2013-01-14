/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/srba.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene

using namespace mrpt::srba;
using namespace std;

// --------------------------------------------------------------------------------
// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
// --------------------------------------------------------------------------------
typedef RBA_Problem<
	kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean3D,          // Parameterization of landmark positions    
	observations::StereoCamera       // Type of observations
	> 
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset (generated with http://code.google.com/p/recursive-world-toolkit/ )
// --------------------------------------------------------------------------------
struct basic_stereo_dataset_entry_t 
{
	unsigned int landmark_id;
	double l_px_x, l_px_y, r_px_x, r_px_y;
};
// Observations for KF#0
basic_stereo_dataset_entry_t  dataset0[] = {
	{ 146,    349.055,    388.990,    294.633,    388.990},
	{ 144,    166.936,    544.592,    112.938,    544.592},
	{  55,    752.822,    474.936,    699.173,    474.936},
	{  39,    426.136,    303.168,    387.707,    303.168},
	{  35,    364.692,    324.945,    328.908,    324.945},
	{  28,    451.591,    337.631,    418.692,    337.631},
	{ 141,    585.145,    423.483,    548.951,   423.483},
	{  84,     67.810,    186.054,     33.441,    186.054},
	{  76,    478.439,    204.754,    448.014,    204.754},
	{  37,    572.299,     78.108,    541.746,     78.108},
	{ 127,     86.285,    486.340,     53.569,    486.340},
	{  59,    591.251,    211.073,    563.092,    211.073},
	{   6,    589.306,    385.881,    561.259,    385.881},
	{ 113,     99.627,    301.455,     72.579,    301.455} };
	
// Observations for KF#1. GT pose: 10 -> 0.502744 -0.099901 0.000000 0.526224 -0.526224 0.472322 -0.472322
basic_stereo_dataset_entry_t  dataset1[] = {
	{  39,    425.337,    429.116,    353.568,   429.116},
	{  35,    306.011,    463.319,    240.591,    463.319},
	{  28,    443.661,    463.791,    389.828,    463.791},
	{  76,    476.466,    248.034,    429.644,    248.034},
	{ 141,    661.388,    604.077,    602.651,    604.077},
	{  37,    609.815,     64.544,    564.845,     64.544},
	{  59,    615.554,    259.433,    576.179,    259.433},
	{ 128,     46.137,    121.924,      7.580,    121.924},
	{   6,    608.662,    502.463,    569.507,    502.463},
	{  86,    170.860,     65.237,    134.819,     65.237},
	{  13,    175.479,    526.105,    140.839,    526.105},
	{  26,    443.565,    211.709,    417.054,    211.709},
	{  77,    294.908,    248.469,    269.542,    248.469},
	{  97,     49.000,    303.886,     19.982,    303.886},
	{  32,    474.699,    223.729,    449.313,    223.729},
	{   3,    499.129,     85.556,    472.420,     85.556},
	{ 105,    130.012,    390.366,    102.778,    390.366},
	{   1,    848.206,    127.293,    815.747,    127.293} };

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.use_robust_kernel = true;
	rba.parameters.std_noise_observations = 0.5; // pixels

	// =========== Topology parameters ===========
	rba.parameters.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.max_tree_depth       = 3;
	rba.parameters.max_optimize_depth   = 3;
	// ===========================================

	// Set camera calib:
	mrpt::utils::TCamera & lc = rba.sensor_params.camera_calib.leftCamera;
	lc.ncols = 1024;
	lc.nrows = 768;
	lc.cx(512);
	lc.cy(384);
	lc.fx(200);
	lc.fy(150);
	lc.dist.setZero();
	rba.sensor_params.camera_calib.rightCamera = lc;
	rba.sensor_params.camera_calib.rightCameraPose.fromString("[0.2 0 0  1 0 0 0]");  // [X Y Z qr qx qy qz]
	
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
	srba::TNewKeyFrameInfo new_kf_info;
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
