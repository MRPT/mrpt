/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
	kf2kf_pose_traits_SE3,                // Parameterization  KF-to-KF poses
	landmark_traits_Euclidean3D,          // Parameterization of landmark positions    
	observation_traits_Cartesian_3D       // Type of observations
	> 
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-cartesian.cfg
// --------------------------------------------------------------------------------
struct basic_euclidean_dataset_entry_t 
{
	unsigned int landmark_id;
	double x,y,z;
};
// Observations for KF#0. Ground truth pose: (xyz,q)=0.000000 0.000000 0.000000 0.996195 0.000000 0.000000 0.087156
basic_euclidean_dataset_entry_t  observations_0[] = {
 {    21,     0.156,    -0.729,     0.000},
 {    23,     0.433,    -1.983,     0.000},
 {    39,     2.655,    -0.362,     0.000},
 {    35,     2.862,    -0.047,     0.000},
 {    20,     1.388,     2.568,     0.000},
 {    28,     3.120,    -0.577,     0.000},
 {    18,     1.787,    -3.011,     0.000},
 {    37,     3.212,    -1.409,     0.000},
 {    45,     1.245,    -3.324,     0.000},
 {     6,     3.690,    -1.656,     0.000},
 {    13,     4.447,     0.581,     0.000},
 {    43,     1.003,    -4.409,     0.000},
 {    30,     1.787,    -4.286,     0.000},
 {    24,     4.634,     1.725,     0.000},
 {     3,     4.703,    -1.610,     0.000},
 };
// Observations for KF#10:(xyz,q)= 1.230806 0.294472 0.000000 0.999999 0.000000 0.000000 0.001357
basic_euclidean_dataset_entry_t  observations_10[] = {
 {    21,    -0.954,    -0.983,     0.000},
 {    39,     1.447,    -0.194,     0.000},
 {    35,     1.597,     0.152,     0.000},
 {    28,     1.941,    -0.327,     0.000},
 {    23,    -0.466,    -2.171,     0.000},
 {    37,     2.174,    -1.130,     0.000},
 {    20,    -0.303,     2.476,     0.000},
 {     6,     2.687,    -1.292,     0.000},
 {    18,     1.044,    -2.952,     0.000},
 {    13,     3.051,     1.041,     0.000},
 {    45,     0.563,    -3.354,     0.000},
 {    24,     3.039,     2.201,     0.000},
 {     3,     3.677,    -1.073,     0.000},
 {    26,     3.786,    -0.662,     0.000},
 {    32,     3.962,    -0.937,     0.000},
 {     8,     3.147,    -2.855,     0.000},
 {     1,     3.046,    -3.034,     0.000},
 {    30,     1.262,    -4.209,     0.000},
 {    12,     4.090,     1.818,     0.000},
 {    43,     0.511,    -4.464,     0.000},
 {     5,     4.565,    -0.498,     0.000},
 {     2,     4.287,     1.885,     0.000},
 {    11,     3.162,    -3.549,     0.000},
 };

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.use_robust_kernel = true;
	rba.parameters.std_noise_pixels = 0.5;  MRPT_TODO("change!")

	// =========== Topology parameters ===========
	rba.parameters.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.max_tree_depth       = 3;
	rba.parameters.max_optimize_depth   = 3;
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
		obs_field.obs.obs_data.pt.x = observations_0[i].x;
		obs_field.obs.obs_data.pt.y = observations_0[i].y;
		obs_field.obs.obs_data.pt.z = observations_0[i].z;
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
	// Define observations of next KF:
	// --------------------------------------------------------------------------------
	list_obs.clear();

	for (size_t i=0;i<sizeof(observations_10)/sizeof(observations_10[0]);i++)
	{
		obs_field.obs.feat_id = observations_10[i].landmark_id;
		obs_field.obs.obs_data.pt.x = observations_10[i].x;
		obs_field.obs.obs_data.pt.y = observations_10[i].y;
		obs_field.obs.obs_data.pt.z = observations_10[i].z;
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
