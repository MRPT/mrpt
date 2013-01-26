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
const double SENSOR_NOISE_STD = 1e-4;
struct basic_range_bearing_dataset_entry_t 
{
	unsigned int landmark_id;
	double range,yaw,pitch;
};

// Observations for KF#0. Ground truth pose: (xyz,q)= 0.000000 0.000000 0.000000 0.995247 0.003802 -0.043453 0.087073
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble(0.995247,0.003802,-0.043453,0.087073));

basic_range_bearing_dataset_entry_t   observations_0[] = {
 {    98,   1.60428511,  -1.11461540,  -0.89550497},
 {   124,   1.58774441,   1.33164997,  -0.19622291},
 {    61,   1.65011270,  -1.66658707,  -0.52575456},
 {    67,   1.72361599,   1.20639193,  -0.18217297},
 {   143,   1.97277071,  -0.40272641,  -1.20129363},
 {   146,   1.91752877,   0.01414273,   0.26177814},
 {    21,   2.08720923,  -1.59226646,   1.21510288},
 {    53,   2.10781646,  -1.20301045,  -0.03318179},
 {   144,   2.26218777,   0.35682263,   0.49373240},
 {   119,   2.53555166,  -0.79764593,  -1.06219197},
 {   118,   2.44996712,   0.80320038,   0.24898747},
 {    55,   2.52130170,  -0.66303164,   0.33183072},
 {    64,   2.60537599,  -1.13439113,   0.23503897},
 {    39,   2.66014719,  -0.13678996,   0.09771111},
 {   107,   2.77534099,   1.19803018,  -0.22831016},
 {    35,   2.84520622,  -0.01664821,   0.14087378},
 {   114,   3.01203485,   0.91495407,  -0.79792115},
 {    20,   2.91895636,   1.08580731,   0.13670457},
 {    58,   2.95699052,  -1.62210638,   0.40731255},
 {    63,   3.03788939,   1.08666798,  -0.83418134},
 {    23,   3.12679854,  -1.25957085,  -0.84521171},
 {    28,   3.15872335,  -0.18561523,   0.16250199},
 {    52,   3.31972811,   0.84537147,  -0.36119838},
 {   141,   3.19456952,  -0.42350035,   0.29714027},
 {    84,   3.39460400,   0.51654507,  -0.11400210},
 {    76,   3.42171672,  -0.23615655,  -0.09196079},
 {    71,   3.67377580,   0.00459775,  -0.54595941},
 {    45,   3.53555514,  -1.21325757,   0.00964582},
 {    68,   3.53881723,  -1.42658605,   0.67617074},
 {    95,   3.50248394,   0.11396022,   0.86402252},
 {    37,   3.76060534,  -0.40227756,  -0.30677018},
 {    18,   3.65036270,  -1.06219533,   0.34592444},
 {    91,   3.72981585,  -1.65868798,  -0.67188426},
 {   100,   3.95072393,   0.86900172,  -0.44090057},
 {   127,   3.76550116,   0.48860876,   0.38555804},
 {   115,   3.90762380,   0.81505803,  -0.17974561},
 {    59,   3.95507519,  -0.43344780,  -0.07465010},
 };
// Observations for KF#10:(xyz,q)= 1.226070 0.293637 0.110099 0.999103 0.000160 -0.042322 0.001049
const mrpt::poses::CPose3DQuat GT_pose10(1.226071, 0.293637, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));
basic_range_bearing_dataset_entry_t  observations_10[] = {
 {   146,   0.88599764,   0.10332230,   0.83802083},
 {    39,   1.51019896,  -0.13524474,   0.33114157},
 {   144,   1.51242063,   0.98270341,   0.94766060},
 {    35,   1.68742826,   0.09858569,   0.39107517},
 {   118,   1.84320369,   1.51134845,   0.40445314},
 {    55,   1.93814338,  -1.01548549,   0.54728509},
 {    28,   2.06846300,  -0.17048891,   0.38844062},
 {    53,   2.11104659,  -1.64093173,   0.01291842},
 {    76,   2.22915966,  -0.23247982,  -0.00936564},
 {    84,   2.31303712,   0.93419033,  -0.06931256},
 {   141,   2.34502530,  -0.54109406,   0.53505398},
 {    64,   2.49028016,  -1.47875109,   0.30062864},
 {    71,   2.49299013,   0.12625731,  -0.72128430},
 {    20,   2.54628814,   1.70789327,   0.19016032},
 {    52,   2.59243040,   1.40249196,  -0.40965579},
 {   114,   2.60044786,   1.73535416,  -0.92388489},
 {    37,   2.65977882,  -0.46313345,  -0.32044475},
 {    59,   2.89638704,  -0.47213869,   0.01141877},
 {   127,   2.93616288,   0.88931513,   0.60152123},
 {    95,   3.04488894,   0.39090932,   1.22500020},
 {   115,   3.09422657,   1.27633160,  -0.16714662},
 {   113,   3.13437144,   0.80859040,   0.20829278},
 {    23,   3.14474638,  -1.69442005,  -0.80209781},
 {   128,   3.16030876,   0.54822353,  -0.21670422},
 {     6,   3.16473231,  -0.46131440,   0.41454174},
 {    86,   3.17553563,   0.35002144,  -0.33416667},
 {   100,   3.22501691,   1.36825688,  -0.49516805},
 {    45,   3.40382066,  -1.40643870,   0.05618249},
 {    18,   3.40978440,  -1.26468831,   0.43312931},
 {    99,   3.47157048,   1.18299147,   0.27180893},
 {    13,   3.48896304,   0.34197115,   0.46908439},
 {    68,   3.71363269,  -1.69773493,   0.66827748},
 {    26,   3.89958095,  -0.17030489,  -0.07981308}, 
};


int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = false; // true
	rba.parameters.srba.std_noise_observations = 0.03; //SENSOR_NOISE_STD;

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

