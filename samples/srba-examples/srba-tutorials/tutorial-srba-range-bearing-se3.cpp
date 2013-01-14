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
#include <mrpt/random.h>

using namespace mrpt::srba;
using namespace mrpt::random;
using namespace std;

// --------------------------------------------------------------------------------
// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
// --------------------------------------------------------------------------------
typedef RBA_Problem<
	kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean3D,          // Parameterization of landmark positions    
	observations::RangeBearing_3D    // Type of observations
	>
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-range-bearing-3d.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-5;
struct basic_range_bearing_dataset_entry_t 
{
	unsigned int landmark_id;
	double range,yaw,pitch;
};

// Observations for KF#0. Ground truth pose: (xyz,q)=0.000000 0.000000 0.000000 0.996195 0.000000 0.000000 0.087156
basic_range_bearing_dataset_entry_t   observations_0[] = {
 {    98,     1.604,    -1.115,    -0.896},
 {   124,     1.588,     1.332,    -0.196},
 {    61,     1.650,    -1.667,    -0.526},
 {    67,     1.724,     1.206,    -0.182},
 {   143,     1.973,    -0.403,    -1.201},
 {   146,     1.918,     0.014,     0.262},
 {    21,     2.087,    -1.592,     1.215},
 {    53,     2.108,    -1.203,    -0.033},
 {   144,     2.262,     0.357,     0.494},
 {   119,     2.536,    -0.798,    -1.062},
 {   118,     2.450,     0.803,     0.249},
 {    55,     2.521,    -0.663,     0.332},
 {    64,     2.605,    -1.134,     0.235},
 {    39,     2.660,    -0.137,     0.098},
 {   107,     2.775,     1.198,    -0.228},
 {    35,     2.845,    -0.017,     0.141},
 {   114,     3.012,     0.915,    -0.798},
 {    20,     2.919,     1.086,     0.137},
 {    58,     2.957,    -1.622,     0.407},
 {    63,     3.038,     1.087,    -0.834},
 {    23,     3.127,    -1.260,    -0.845},
 {    28,     3.159,    -0.186,     0.163},
 {    52,     3.320,     0.845,    -0.361},
 {   141,     3.195,    -0.424,     0.297},
 {    84,     3.395,     0.517,    -0.114},
 {    76,     3.422,    -0.236,    -0.092},
 {    71,     3.674,     0.005,    -0.546},
 {    45,     3.536,    -1.213,     0.010},
 {    68,     3.539,    -1.427,     0.676},
 {    95,     3.502,     0.114,     0.864},
 {    37,     3.761,    -0.402,    -0.307},
 {    18,     3.650,    -1.062,     0.346},
 {    91,     3.730,    -1.659,    -0.672},
 {   100,     3.951,     0.869,    -0.441},
 {   127,     3.766,     0.489,     0.386},
 {   115,     3.908,     0.815,    -0.180},
 {    59,     3.955,    -0.433,    -0.075},
 };
// Observations for KF#10:(xyz,q)= 1.230806 0.294472 0.000000 0.999999 0.000000 0.000000 0.001357
basic_range_bearing_dataset_entry_t  observations_10[] = {
 {   146,     0.886,     0.103,     0.838},
 {    39,     1.510,    -0.135,     0.331},
 {   144,     1.512,     0.983,     0.948},
 {    35,     1.687,     0.099,     0.391},
 {   118,     1.843,     1.511,     0.404},
 {    55,     1.938,    -1.015,     0.547},
 {    28,     2.068,    -0.170,     0.388},
 {    53,     2.111,    -1.641,     0.013},
 {    76,     2.229,    -0.232,    -0.009},
 {    84,     2.313,     0.934,    -0.069},
 {   141,     2.345,    -0.541,     0.535},
 {    64,     2.490,    -1.479,     0.301},
 {    71,     2.493,     0.126,    -0.721},
 {    20,     2.546,     1.708,     0.190},
 {    52,     2.592,     1.402,    -0.410},
 {   114,     2.600,     1.735,    -0.924},
 {    37,     2.660,    -0.463,    -0.320},
 {    59,     2.896,    -0.472,     0.011},
 {   127,     2.936,     0.889,     0.602},
 {    95,     3.045,     0.391,     1.225},
 {   115,     3.094,     1.276,    -0.167},
 {   113,     3.134,     0.809,     0.208},
 {    23,     3.145,    -1.694,    -0.802},
 {   128,     3.160,     0.548,    -0.217},
 {     6,     3.165,    -0.461,     0.415},
 {    86,     3.176,     0.350,    -0.334},
 {   100,     3.225,     1.368,    -0.495},
 {    45,     3.404,    -1.406,     0.056},
 {    18,     3.410,    -1.265,     0.433},
 {    99,     3.472,     1.183,     0.272},
 {    13,     3.489,     0.342,     0.469},
 {    68,     3.714,    -1.698,     0.668},
 {    26,     3.900,    -0.170,    -0.080},
};


int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.use_robust_kernel = true;
	rba.parameters.std_noise_observations = 0.03; //SENSOR_NOISE_STD;

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
		obs_field.obs.obs_data.range = observations_0[i].range + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.yaw = observations_0[i].yaw + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pitch = observations_0[i].pitch + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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

