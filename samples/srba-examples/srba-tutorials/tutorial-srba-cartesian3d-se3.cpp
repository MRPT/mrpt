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
	observations::Cartesian_3D       // Type of observations
	> 
	my_srba_t;

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-cartesian.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-8;
struct basic_euclidean_dataset_entry_t 
{
	unsigned int landmark_id;
	double x,y,z;
};
// Observations for KF#0. Ground truth pose: (xyz,q)=0.000000 0.000000 0.000000 0.996195 0.000000 0.000000 0.087156
const mrpt::poses::CPose3DQuat GT_pose0(0,0,0, mrpt::math::CQuaternionDouble(0.995247,0.003802,-0.043453,0.087073));
basic_euclidean_dataset_entry_t  observations_0[] = {
 {    98,   0.44179076,  -0.90032543,   1.25218439},
 {   124,   0.36887709,   1.51295633,   0.30955637},
 {    61,  -0.13650907,  -1.42071439,   0.82813513},
 {    67,   0.60411947,   1.58378786,   0.31226236},
 {   143,   0.65546905,  -0.27923683,   1.83962262},
 {   146,   1.85201587,   0.02619432,  -0.49625361},
 {    21,  -0.01560438,  -0.72668324,  -1.95656085},
 {    53,   0.75744877,  -1.96577507,   0.06992830},
 {   144,   1.86654080,   0.69580840,  -1.07208657},
 {   119,   0.86231273,  -0.88369854,   2.21461415},
 {   118,   1.64881194,   1.70858741,  -0.60372770},
 {    55,   1.87871121,  -1.46722478,  -0.82137561},
 {    64,   1.07097295,  -2.29627194,  -0.60674226},
 {    39,   2.62272811,  -0.36101742,  -0.25951251},
 {   107,   0.98453078,   2.51766692,   0.62814808},
 {    35,   2.81663034,  -0.04689619,  -0.39949054},
 {    20,   1.34811775,   2.55825048,  -0.39779297},
 {    58,  -0.13924954,  -2.71150220,  -1.17139161},
 };
// Observations for KF#10:(xyz,q)= 1.230806 0.294472 0.000000 0.999999 0.000000 0.000000 0.001357
const mrpt::poses::CPose3DQuat GT_pose10(1.226071, 0.293637, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));
basic_euclidean_dataset_entry_t  observations_10[] = {
 {   146,   0.58951359,   0.06112756,  -0.65858034},
 {    39,   1.41511047,  -0.19256187,  -0.49100010},
 {   144,   0.48965781,   0.73434415,  -1.22816494},
 {    67,  -0.90853861,   1.37461725,   0.27968159},
 {    35,   1.55245133,   0.15354732,  -0.64321834},
 {   124,  -1.12818359,   1.26409464,   0.29563349},
 {    98,  -0.64338581,  -1.12385762,   1.20105908},
 {   118,   0.10067360,   1.69149637,  -0.72533028},
 {   143,  -0.54019742,  -0.48152216,   1.78170216},
 {    55,   0.87255879,  -1.40636428,  -1.00855320},
 {    28,   1.88660884,  -0.32479904,  -0.78342141},
 {    53,  -0.14792600,  -2.10568108,  -0.02727064},
 {    61,  -1.12336571,  -1.73181968,   0.81636388},
 {    76,   2.16909508,  -0.51355652,   0.02087719},
 {    84,   1.37172789,   1.85548874,   0.16019420},
 {   141,   1.72910829,  -1.03905231,  -1.19569904},
 {   119,  -0.23312570,  -1.04921366,   2.13211167},
 {    21,  -1.11968588,  -0.98349014,  -1.97883007},
 {    64,   0.21862851,  -2.36852353,  -0.73742356},
 {    71,   1.85722883,   0.23574279,   1.64624521},
 {   107,  -0.69438040,   2.35901117,   0.57829803},
 {    20,  -0.34172377,   2.47692710,  -0.48129002},
 {    52,   0.39832786,   2.34432774,   1.03254850},
 {   114,  -0.25675884,   1.54618058,   2.07502518},
 {    37,   2.25845709,  -1.12777772,   0.83780026},
 {    63,  -0.60857381,   1.62555091,   2.19917666},
 {    59,   2.57934672,  -1.31716782,  -0.03307247},
 {   127,   1.52496454,   1.88008741,  -1.66156676},
 };

int main(int argc, char**argv)
{
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = true;
	rba.parameters.srba.std_noise_observations = 0.1;

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
