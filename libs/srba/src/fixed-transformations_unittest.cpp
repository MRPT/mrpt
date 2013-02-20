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

#include <gtest/gtest.h>

using namespace mrpt::srba;
using namespace mrpt::random;
using namespace std;

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
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/ 
//  and the script: tutorials_dataset-cartesian.cfg
// --------------------------------------------------------------------------------
struct basic_euclidean_dataset_entry_t 
{
	unsigned int landmark_id;
	double x,y,z;
};

basic_euclidean_dataset_entry_t  dummy_obs[] = {
 {    0,   5 , 4 ,  3 },
 {    1,   1 , 7 ,  8 },
 {    2,  -2 ,-3 , -1 },
 {    3,   4 , -2,  9 },
};

template <bool INVERSE_INCR>
void run_test(const mrpt::poses::CPose3D &incr)
{
	my_srba_t rba;     //  Create an empty RBA problem

	rba.get_time_profiler().disable();

	// --------------------------------------------------------------------------------
	// Set parameters 
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 0 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = false;

	// =========== Topology parameters ===========
	rba.parameters.srba.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.srba.max_tree_depth       = 3;
	rba.parameters.srba.max_optimize_depth   = 3;
	// ===========================================

	// --------------------------------------------------------------------------------
	// Define observations of KF #0:
	// --------------------------------------------------------------------------------
	my_srba_t::new_kf_observations_t  list_obs;
	my_srba_t::new_kf_observation_t   obs_field;

	obs_field.is_fixed = false;   // Landmarks have unknown relative positions (i.e. treat them as unknowns to be estimated)
	obs_field.is_unknown_with_init_val = false; // We don't have any guess on the initial LM position (will invoke the inverse sensor model)

	for (size_t i=0;i<sizeof(dummy_obs)/sizeof(dummy_obs[0]);i++)
	{
		obs_field.obs.feat_id = dummy_obs[i].landmark_id;
		obs_field.obs.obs_data.pt.x = dummy_obs[i].x;
		obs_field.obs.obs_data.pt.y = dummy_obs[i].y;
		obs_field.obs.obs_data.pt.z = dummy_obs[i].z;
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

	// --------------------------------------------------------------------------------
	// Define observations of next KF:
	// --------------------------------------------------------------------------------
	list_obs.clear();

	for (size_t i=0;i<sizeof(dummy_obs)/sizeof(dummy_obs[0]);i++)
	{
		obs_field.obs.feat_id = dummy_obs[i].landmark_id;
		obs_field.obs.obs_data.pt.x = dummy_obs[i].x;
		obs_field.obs.obs_data.pt.y = dummy_obs[i].y;
		obs_field.obs.obs_data.pt.z = dummy_obs[i].z;

		if (INVERSE_INCR)
			incr.inverseComposePoint(
				obs_field.obs.obs_data.pt.x, obs_field.obs.obs_data.pt.y, obs_field.obs.obs_data.pt.z,
				obs_field.obs.obs_data.pt.x, obs_field.obs.obs_data.pt.y, obs_field.obs.obs_data.pt.z);
		else
			incr.composePoint(obs_field.obs.obs_data.pt, obs_field.obs.obs_data.pt);

		list_obs.push_back( obs_field );
	}
	
	//  Here happens the main stuff: create Key-frames, build structures, run optimization, etc.
	//  ============================================================================================
	rba.define_new_keyframe(
		list_obs,      // Input observations for the new KF
		new_kf_info,   // Output info
		true           // Also run local optimization?
		);

	mrpt::poses::CPose3D estIncr = rba.get_k2k_edges()[0].inv_pose;
	if (INVERSE_INCR)
		estIncr.inverse();

	EXPECT_NEAR( (incr.getHomogeneousMatrixVal()-estIncr.getHomogeneousMatrixVal()).array().abs().sum(),0, 1e-3)
		<< "=> Ground truth: " << incr << " Inverse: " << (INVERSE_INCR ? "YES":"NO") << endl
		<< "=> inv_pose of KF-to-KF edge #0 (relative pose of KF#0 wrt KF#1):\n" << estIncr << endl
		<< "=> Optimization error: " << new_kf_info.optimize_results.total_sqr_error_init << " -> " << new_kf_info.optimize_results.total_sqr_error_final << endl;
}

const double test_fixed_transfs[][6] = {
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	//
	{ 1,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	1,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	0,		1,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	//
	{-1,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	-1,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	0,		-1,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(0) },
	// 45deg
	{ 0,	0,		0,	DEG2RAD(45),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(45),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(45) },
	// -45deg
	{ 0,	0,		0,	DEG2RAD(-45),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(-45),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(-45) },
	// 90 deg
	{ 0,	0,		0,	DEG2RAD(90),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(90),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(90) },
	// -90deg
	{ 0,	0,		0,	DEG2RAD(-90),	DEG2RAD(0),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(-90),	DEG2RAD(0) },
	{ 0,	0,		0,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(-90) },
	// 45deg+displ
	{ 1,	2,		3,	DEG2RAD(45),	DEG2RAD(0),	DEG2RAD(0) },
	{ 1,	2,		3,	DEG2RAD(0),	DEG2RAD(45),	DEG2RAD(0) },
	{ 1,	2,		3,	DEG2RAD(0),	DEG2RAD(0),	DEG2RAD(45) },
	// full 6D
	{ 1,	2,		3,	DEG2RAD(10),	DEG2RAD(20),	DEG2RAD(30) },
};


TEST(MiniProblems,FixedTransformations)
{
	for (size_t i=0;i<sizeof(test_fixed_transfs)/sizeof(test_fixed_transfs[0]);i++)
	{
		const double *p = test_fixed_transfs[i];

		const mrpt::poses::CPose3D incr(p[0],p[1],p[2],p[3],p[4],p[5]);
		run_test<false>(incr);
		run_test<true>(incr);
	}
}
