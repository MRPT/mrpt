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
		typedef options::sensor_pose_on_robot_none sensor_pose_on_robot_t;
		typedef options::observation_noise_identity   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
		typedef options::solver_LM_schur_dense_cholesky      solver_t;
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
 {   124,   0.47137417,   1.15859855,  -0.49727044},
 {    98,  -0.17789146,  -0.53651374,   1.36342535},
 {    67,   0.70678360,   1.21719806,  -0.45411808},
 {   143,  -0.07370551,   0.28585934,   1.63404845},
 {    61,  -0.67001603,  -1.18419781,   1.06964358},
 {   146,   1.92140073,  -0.57285731,  -0.04161594},
 {    53,   0.35504924,  -2.04531849,   0.96626790},
 {   144,   2.24265800,  -0.25154418,  -0.80396101},
 {   118,   2.03974538,   0.87163457,  -0.93657980},
 {    21,   0.51523097,  -1.88549822,  -1.47525074},
 {   107,   1.11472224,   2.18476013,  -0.49254479},
 {   114,   0.73529133,   2.14473289,   1.22408807},
 {    63,   0.40815147,   2.32039303,   1.13685256},
 {    39,   2.51298804,  -0.82203149,   0.56003873},
 {    55,   1.81390599,  -2.04988706,   0.36206198},
 {    20,   1.82095162,   1.72844780,  -1.23686783},
 {    64,   0.83275514,  -2.66449845,   0.65614084},
 {    35,   2.79774149,  -0.61423463,   0.36623363},
 {    52,   1.92360103,   2.24506316,   0.36922611},
 };
// Observations for KF#10:
basic_euclidean_dataset_entry_t  observations_10_displ[] = {
 {   146,   0.75819641,  -0.56412427,  -0.53356036},
 {    67,  -0.73470065,   1.00894605,  -0.91626017},
 {    39,   1.42350566,  -0.69440032,   0.02719219},
 {   124,  -0.96145425,   0.91493785,  -0.93768195},
 {   144,   0.97045031,  -0.23938492,  -1.33175270},
 {    35,   1.65914153,  -0.45797992,  -0.19731973},
 {   118,   0.59468239,   0.83460554,  -1.49186128},
 {    84,   1.49482950,   1.41831478,  -0.36241681},
 {    28,   1.93827415,  -0.94000082,   0.02600561},
 {    76,   1.89380462,  -0.72360191,   0.87070561},
 {    55,   0.90036971,  -2.01861052,  -0.06410921},
 {    71,   1.17155735,   0.69575350,   1.74495977},
 {   114,  -0.71758153,   2.01564777,   0.71584522},
 {    52,   0.37543292,   2.24207016,  -0.23651364},
 {   107,  -0.47801863,   2.02190155,  -1.02627098},
 {    53,  -0.49373385,  -2.19314814,   0.65655752},
 {    61,  -1.62395680,  -1.48439853,   0.80901815},
 {    20,   0.22997715,   1.63448423,  -1.80707024},
 {    63,  -1.07266792,   2.13768663,   0.64870970},
 {   141,   1.81697891,  -1.76692173,  -0.05415843},
 {    37,   1.59687150,  -0.88025216,   1.84019412},
 {    64,   0.04547884,  -2.75217780,   0.33329130},
 {   115,   1.09726789,   2.51545483,  -0.72721361},
 {    21,  -0.54242948,  -2.13719493,  -1.79430191},
 {   100,   0.43510915,   2.86426353,   0.04364043},
 {   128,   2.44512710,   1.46712256,   0.64692397},
 {    59,   2.16078023,  -1.45079228,   1.33645906},
 {    86,   2.39687467,   1.12879188,   1.26682522},
 {   127,   2.26373009,   0.58683964,  -1.79780544},
 };



struct TEST_DATASET1
{
	struct my_srba_options
	{
		typedef options::sensor_pose_on_robot_se3 sensor_pose_on_robot_t;
		typedef options::observation_noise_identity   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
		typedef options::solver_LM_schur_dense_cholesky      solver_t;
	};

	static basic_euclidean_dataset_entry_t * getData0(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#0: 0.000000 0.000000 0.000000 0.995247 0.003802 -0.043453 0.087073
		GT_pose = mrpt::poses::CPose3DQuat(0,0,0, mrpt::math::CQuaternionDouble(0.995247,0.003802,-0.043453,0.087073));

		N= sizeof(observations_0_displ)/sizeof(observations_0_displ[0]);
		return observations_0_displ;
	}

	static basic_euclidean_dataset_entry_t * getData1(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#10: 1.226072 0.293638 0.110099 0.999103 0.000160 -0.042322 0.001049
		GT_pose = mrpt::poses::CPose3DQuat(1.226072, 0.293638, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));

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
	typedef RbaEngine<
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

	rba.parameters.obs_noise.std_noise_observations = 0.1;

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
	typename my_srba_t::TNewKeyFrameInfo new_kf_info;
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
	const mrpt::poses::CPose3D P = -rba.get_k2k_edges()[0]
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
		->
#else
		.
#endif
		inv_pose;

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
