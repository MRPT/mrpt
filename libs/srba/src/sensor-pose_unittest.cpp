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

// --------------------------------------------------------------------------------
// A test dataset. Generated with http://code.google.com/p/recursive-world-toolkit/
//  and the script: tutorials_dataset-cartesian.cfg
// --------------------------------------------------------------------------------
const double SENSOR_NOISE_STD = 1e-9;
struct basic_euclidean_dataset_entry_t
{
	unsigned int landmark_id;
	double x,y,z;
};


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

struct TEST_DATASET0
{
	struct my_srba_options
	{
		typedef sensor_pose_on_robot_none sensor_pose_on_robot_t;
	};

	static basic_euclidean_dataset_entry_t * getData0(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#0. Ground truth pose: (xyz,q)=0.000000 0.000000 0.000000 0.995247 0.003802 -0.043453 0.087073
		GT_pose = mrpt::poses::CPose3DQuat(0,0,0, mrpt::math::CQuaternionDouble(0.995247,0.003802,-0.043453,0.087073));

		N= sizeof(observations_0)/sizeof(observations_0[0]);
		return observations_0;
	}

	static basic_euclidean_dataset_entry_t * getData1(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#10: 1.226072 0.293638 0.110099 0.999103 0.000160 -0.042322 0.001049
		GT_pose = mrpt::poses::CPose3DQuat(1.226072, 0.293638, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));

		N= sizeof(observations_10)/sizeof(observations_10[0]);
		return observations_10;
	}

	template <class RBA>
	static void setExtraOptions(RBA &rba)
	{
	}
};

// Observations for KF#0
basic_euclidean_dataset_entry_t  observations_0_displ[] = {
 {   124,   0.18076884,   1.21566289,  -0.13450912},
 {    67,   0.41679687,   1.28451620,  -0.15074544},
 {    98,   0.22269495,  -1.19571229,   0.83536322},
 {   143,   0.43581449,  -0.56718263,   1.40097758},
 {    61,  -0.35618174,  -1.71764060,   0.46055450},
 {   146,   1.66057599,  -0.30083055,  -1.03924990},
 {   119,   0.63359191,  -1.17128656,   1.76880368},
 {   118,   1.47307125,   1.38844693,  -1.15197136},
 {   107,   0.80238105,   2.22205429,   0.12568550},
 {   144,   1.68703132,   0.36406450,  -1.62647479},
 {    53,   0.54170776,  -2.28266181,  -0.36256130},
 {   114,   1.07703890,   1.38378622,   1.64695736},
 {    63,   0.74440558,   1.52825627,   1.76515415},
 {    21,  -0.19982610,  -1.05635357,  -2.35216166},
 {    39,   2.42604241,  -0.69402512,  -0.85619018},
 {    20,   1.17740730,   2.24669251,  -0.93249499},
 {    55,   1.67762948,  -1.80403200,  -1.34944732},
 {    52,   1.87167416,   2.02355593,   0.59235572},
 {    35,   2.62432721,  -0.38233014,  -1.01541913},
 {    64,   0.85970812,  -2.62571287,  -1.06181611},
 {    23,   0.39559670,  -2.26309411,   1.92458002},
 };
// Observations for KF#10:
basic_euclidean_dataset_entry_t  observations_10_displ[] = {
{   146,   0.39847745,  -0.25512687,  -1.05142697},
 {    67,  -1.09779937,   1.08255843,  -0.14505799},
 {    39,   1.22002907,  -0.51383915,  -0.87205988},
 {   144,   0.31046998,   0.41191492,  -1.63015688},
 {    35,   1.36197131,  -0.17080897,  -1.02700236},
 {   143,  -0.76141787,  -0.75819164,   1.38317013},
 {   118,  -0.07539158,   1.37844756,  -1.14310731},
 {    98,  -0.86412131,  -1.40666155,   0.80929919},
 {    84,   1.18770260,   1.54231112,  -0.24624169},
 {    71,   1.64346296,  -0.06327428,   1.26439187},
 {   114,  -0.46338356,   1.27043120,   1.65486696},
 {    52,   0.20952348,   2.05012171,   0.60977658},
 {    28,   1.69339191,  -0.65368776,  -1.15785281},
 {    76,   1.96578238,  -0.83502739,  -0.34838169},
 {   119,  -0.46299775,  -1.32417924,   1.74366559},
 {   107,  -0.87819831,   2.06864919,   0.14384906},
 {    55,   0.67235590,  -1.72916392,  -1.38056713},
 {    63,  -0.81577334,   1.35433420,   1.77431877},
 {    20,  -0.51344974,   2.17057560,  -0.91330144},
 {    61,  -1.34533323,  -2.01512853,   0.42694857},
 {    53,  -0.36441209,  -2.40764102,  -0.40172617},
 {    37,   2.04122176,  -1.43996857,   0.47682835},
 {   141,   1.53399072,  -1.37154367,  -1.56308255},
 {   115,   0.70710736,   2.61499287,   0.09025544},
 {   100,   0.38055816,   2.49002104,   1.10617144},
 {   128,   2.44235141,   1.29088210,   0.28934802},
 {    86,   2.61737708,   0.71395842,   0.66029084},
 {    64,   0.00725139,  -2.68228119,  -1.10472302},
 {   113,   1.94509444,   1.88862743,  -1.05099583},
 {    59,   2.36954856,  -1.64273537,  -0.38824114},
 {   127,   1.36021674,   1.54333431,  -2.06644425},
 };



struct TEST_DATASET1
{
	struct my_srba_options
	{
		typedef sensor_pose_on_robot_se3 sensor_pose_on_robot_t;
	};

	static basic_euclidean_dataset_entry_t * getData0(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#0: 0.109785 0.323986 0.415909 0.995033 0.009212 -0.037701 0.091664
		GT_pose = mrpt::poses::CPose3DQuat(0.109785,0.323986,0.415909, mrpt::math::CQuaternionDouble(0.995033,0.009212,-0.037701,0.091664));

		N= sizeof(observations_0_displ)/sizeof(observations_0_displ[0]);
		return observations_0_displ;
	}

	static basic_euclidean_dataset_entry_t * getData1(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#10: 1.390895 0.593890 0.525649 0.999279 0.006052 -0.037057 0.005634
		GT_pose = mrpt::poses::CPose3DQuat( 1.390895,0.593890,0.525649, mrpt::math::CQuaternionDouble(0.999279,0.006052,-0.037057,0.005634));

		N= sizeof(observations_10_displ)/sizeof(observations_10_displ[0]);
		return observations_10_displ;
	}

	template <class RBA>
	static void setExtraOptions(RBA &rba)
	{
		const mrpt::poses::CPose3D sensorPoseOnRobot(0.1, 0.2,0.3,0.1745329251994328, 0.3490658503988660, 0.5235987755982987);
		rba.parameters.sensor_pose.relative_pose = sensorPoseOnRobot;
	}


};


template <class DATASET>
void run_test()
{
	// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
	typedef RBA_Problem<
		kf2kf_poses::SE3,                // Parameterization  KF-to-KF poses
		landmarks::Euclidean3D,          // Parameterization of landmark positions
		observations::Cartesian_3D,       // Type of observations
		typename DATASET::my_srba_options
		>
		my_srba_t;

	my_srba_t rba;     //  Create an empty RBA problem

	rba.get_time_profiler().disable();

	// --------------------------------------------------------------------------------
	// Set parameters
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 0 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = true;
	rba.parameters.srba.std_noise_observations = 0.1;

	// =========== Topology parameters ===========
	rba.parameters.srba.edge_creation_policy = mrpt::srba::ecpICRA2013;
	rba.parameters.srba.max_tree_depth       = 3;
	rba.parameters.srba.max_optimize_depth   = 3;
	// ===========================================

	DATASET::setExtraOptions(rba);


	size_t N0,N1;
	mrpt::poses::CPose3DQuat GT0,GT1;

	const basic_euclidean_dataset_entry_t *d0 = DATASET::getData0(N0,GT0);
	const basic_euclidean_dataset_entry_t *d1 = DATASET::getData1(N1,GT1);


	// --------------------------------------------------------------------------------
	// Define observations of KF #0:
	// --------------------------------------------------------------------------------
	typename my_srba_t::new_kf_observations_t  list_obs;
	typename my_srba_t::new_kf_observation_t   obs_field;

	obs_field.is_fixed = false;   // Landmarks have unknown relative positions (i.e. treat them as unknowns to be estimated)
	obs_field.is_unknown_with_init_val = false; // We don't have any guess on the initial LM position (will invoke the inverse sensor model)

	for (size_t i=0;i<N0;i++)
	{
		obs_field.obs.feat_id = d0[i].landmark_id;
		obs_field.obs.obs_data.pt.x = d0[i].x + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.y = d0[i].y + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.z = d0[i].z + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
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
	for (size_t i=0;i<N1;i++)
	{
		obs_field.obs.feat_id = d1[i].landmark_id;
		obs_field.obs.obs_data.pt.x = d1[i].x + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.y = d1[i].y + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		obs_field.obs.obs_data.pt.z = d1[i].z + randomGenerator.drawGaussian1D(0,SENSOR_NOISE_STD);
		list_obs.push_back( obs_field );
	}

	rba.define_new_keyframe(
		list_obs,      // Input observations for the new KF
		new_kf_info,   // Output info
		true           // Also run local optimization?
		);

	// Compare to ground truth:
	// Relative pose of KF#1 wrt KF#0:
	const mrpt::poses::CPose3D P = -rba.get_k2k_edges()[0].inv_pose;
	const mrpt::poses::CPose3D P_GT = GT1-GT0;

	EXPECT_NEAR(0, (P.getAsVectorVal()-P_GT.getAsVectorVal()).array().abs().sum(), 1e-2 )
		<< "P : " << P << endl
		<< "GT: " << P_GT << endl;
}


TEST(MiniProblems,SensorAtRobot_vs_SensorDisplaced)
{
	run_test<TEST_DATASET0>();
	run_test<TEST_DATASET1>();
}
