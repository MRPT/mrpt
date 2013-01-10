/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
	kf2kf_pose_traits_SE3,                // Parameterization  KF-to-KF poses
	landmark_traits_Euclidean3D,          // Parameterization of landmark positions    
	observation_traits_MonocularCamera    // Type of observations
	> 
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-monocular.cfg
// --------------------------------------------------------------------------------
struct basic_monocular_dataset_entry_t 
{
	unsigned int landmark_id;
	double px_x, px_y; 
};
// Observations for KF#0
basic_monocular_dataset_entry_t dataset0[] = {
	{ 146,    397.171,    353.591},
	{ 144,    325.444,    414.874},
	{ 118,    192.749,    373.232},
	{  55,    556.195,    387.440},
	{  39,    427.530,    319.789},
	{  35,    403.330,    328.367},
	{  20,     20.471,    359.015},
	{  28,    437.555,    333.363},
	{  52,    174.448,    186.120},
	{ 141,    490.155,    367.176},
	{  84,    286.403,    273.664},
	{  76,    448.129,    281.029},
	{  71,    399.080,    178.487},
	{  95,    377.109,    535.739},
	{  37,    485.096,    231.150},
	{  18,    758.729,    448.029},
	{ 100,    163.415,    153.814},
	{ 127,    293.680,    391.932},
	{ 115,    187.770,    247.011},
	{  59,    492.560,    283.518},
	{   6,    491.794,    352.366},
	{ 113,    298.934,    319.115} };
// Observations for KF#10:
basic_monocular_dataset_entry_t dataset1[] = {
	{ 146,    379.262,    523.432},
	{  39,    427.215,    369.394},
	{  35,    380.219,    382.865},
	{  55,    722.354,    531.172},
	{  28,    434.432,    383.051},
	{  76,    447.352,    298.075},
	{  84,    129.467,    276.643},
	{ 141,    520.184,    438.302},
	{  71,    374.613,    122.720},
	{  37,    499.872,    225.808},
	{  59,    502.132,    302.564},
	{ 127,    153.425,    517.916},
	{ 113,    190.501,    361.217},
	{ 128,    277.867,    248.407},
	{   6,    499.417,    398.282},
	{  86,    326.989,    226.080},
	{  13,    328.809,    407.593},
	{  26,    434.394,    283.769} };

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.use_robust_kernel = true;
	rba.parameters.std_noise_pixels = 0.5;

	// =========== Topology parameters ===========
	rba.parameters.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.max_tree_depth       = 3;
	rba.parameters.max_optimize_depth   = 3;
	// ===========================================

	// Set camera calib:
	mrpt::utils::TCamera & c = rba.sensor_params.camera_calib;
	c.ncols = 800;
	c.nrows = 640;
	c.cx(400);
	c.cy(320);
	c.fx(200);
	c.fy(200);
	c.dist.setZero();
	
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
		obs_field.obs.obs_data.px.x = dataset0[i].px_x;
		obs_field.obs.obs_data.px.y = dataset0[i].px_y;
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
		obs_field.obs.obs_data.px.x = dataset1[i].px_x;
		obs_field.obs.obs_data.px.y = dataset1[i].px_y;
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
