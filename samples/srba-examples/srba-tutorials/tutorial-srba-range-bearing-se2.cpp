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
struct my_srba_options
{
	typedef options::sensor_pose_on_robot_none      sensor_pose_on_robot_t;  // The sensor pose coincides with the robot pose
	typedef options::observation_noise_identity     obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
	typedef options::solver_LM_schur_dense_cholesky solver_t;
};

typedef RbaEngine<
	kf2kf_poses::SE2,                // Parameterization  KF-to-KF poses
	landmarks::Euclidean2D,          // Parameterization of landmark positions    
	observations::RangeBearing_2D,   // Type of observations
	my_srba_options                  // Other options
	>
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-range-bearing-2d.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-3;
struct basic_range_bearing_dataset_entry_t 
{
	unsigned int landmark_id;
	double range,yaw,pitch;
};

// Observations for KF#0. Ground truth pose: (xyz,q)= 0.000000 0.000000 0.000000 0.996195 0.000000 0.000000 0.087156
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble(0.996195,0.000000,0.000000,0.087156));

basic_range_bearing_dataset_entry_t   observations_0[] = {
 {    21,   0.74585171,  -1.36075146,   0.00000000},
 {    23,   2.02948709,  -1.35571420,   0.00000000},
 {    39,   2.68006543,  -0.13563474,   0.00000000},
 {    35,   2.86272739,  -0.01644496,   0.00000000},
 {    20,   2.91915698,   1.07525990,   0.00000000},
 {    28,   3.17296903,  -0.18301471,   0.00000000},
 {    18,   3.50125002,  -1.03517885,   0.00000000},
 {    37,   3.50752367,  -0.41336794,   0.00000000},
 {    45,   3.54994055,  -1.21247025,   0.00000000},
  };
// Observations for KF#10:(xyz,q)= 1.230805 0.294472 0.000000 0.999999 0.000000 0.000000 0.001357
const mrpt::poses::CPose3DQuat GT_pose10(1.230805, 0.294472,0.000000, mrpt::math::CQuaternionDouble(0.999999,0.000000,0.000000,0.001357));
basic_range_bearing_dataset_entry_t  observations_10[] = {
 {    39,   1.45968442,  -0.13342076,   0.00000000},
 {    35,   1.60384967,   0.09483688,   0.00000000},
 {    28,   1.96846287,  -0.16671414,   0.00000000},
 {    37,   2.45029433,  -0.47938282,   0.00000000},
 {    20,   2.49487594,   1.69256088,   0.00000000},
 {     6,   2.98119279,  -0.44809197,   0.00000000},
 {    18,   3.13128847,  -1.23093971,   0.00000000},
 {    13,   3.22347598,   0.32892426,   0.00000000},
 {    45,   3.40078719,  -1.40434357,   0.00000000},
 {    24,   3.75211822,   0.62688236,   0.00000000},
 {     3,   3.83093551,  -0.28399202,   0.00000000},
 {    26,   3.84323530,  -0.17313155,   0.00000000},
};


int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = false; // true

	rba.parameters.obs_noise.std_noise_observations = 0.05; //SENSOR_NOISE_STD;

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
		obs_field.obs.obs_data.range = observations_0[i].range + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.yaw = observations_0[i].yaw + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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

