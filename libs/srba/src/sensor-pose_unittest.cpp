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
		// Observations for KF#10
		GT_pose = mrpt::poses::CPose3DQuat(1.226071, 0.293637, 0.110099, mrpt::math::CQuaternionDouble(0.999103,0.000160,-0.042322,0.001049));

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
 {   124,   0.43882989,   1.16488002,  -0.48708068},
 {    98,  -0.20988182,  -0.52079106,   1.37181119},
 {    67,   0.66715958,   1.23290581,  -0.42706862},
 {   143,  -0.11170940,   0.30721739,   1.65686786},
 {    61,  -0.68493162,  -1.18973143,   1.03729722},
 {   146,   1.84435060,  -0.49915114,   0.07417101},
 {    53,   0.31273302,  -2.01090656,   0.99900843},
 {   144,   2.16400357,  -0.17846491,  -0.68422197},
 {   118,   1.96758664,   0.93150891,  -0.83209261},
 {    21,   0.49262687,  -1.88340387,  -1.48916344},
 {   107,   1.06354580,   2.21238988,  -0.43781966},
 {   114,   0.67772688,   2.18473559,   1.29384138},
 {    63,   0.36057498,   2.34608480,   1.18286880},
 {    39,   2.41342663,  -0.71576375,   0.72931074},
 {    55,   1.73656316,  -1.96946535,   0.47824470},
 {    20,   1.75752997,   1.77272771,  -1.15299845},
 {    64,   0.78037933,  -2.61487009,   0.71275047},
 {    35,   2.69199929,  -0.50080406,   0.55027536},
 };
// Observations for KF#10:
basic_euclidean_dataset_entry_t  observations_10_displ[] = {
 {   146,   0.75819714,  -0.56412417,  -0.53356012},
 {    67,  -0.73469980,   1.00894626,  -0.91625995},
 {    39,   1.42350635,  -0.69440028,   0.02719246},
 {   124,  -0.96145341,   0.91493809,  -0.93768173},
 {   144,   0.97045108,  -0.23938481,  -1.33175244},
 {    35,   1.65914225,  -0.45797988,  -0.19731945},
 {   118,   0.59468325,   0.83460568,  -1.49186101},
 {    84,   1.49483036,   1.41831483,  -0.36241649},
 {    28,   1.93827482,  -0.94000081,   0.02600589},
 {    76,   1.89380528,  -0.72360192,   0.87070589},
 {    55,   0.90037031,  -2.01861044,  -0.06410899},
 {    71,   1.17155808,   0.69575352,   1.74496006},
 {   114,  -0.71758067,   2.01564795,   0.71584547},
 {    52,   0.37543383,   2.24207028,  -0.23651335},
 {   107,  -0.47801771,   2.02190175,  -1.02627073},
 {    53,  -0.49373329,  -2.19314798,   0.65655768},
 {    61,  -1.62395620,  -1.48439829,   0.80901829},
 {    20,   0.22997807,   1.63448439,  -1.80706996},
 {    63,  -1.07266705,   2.13768684,   0.64870994},
 {   141,   1.81697953,  -1.76692171,  -0.05415817},
 {    37,   1.59687211,  -0.88025217,   1.84019439},
 {    64,   0.04547938,  -2.75217767,   0.33329147},
 {   115,   1.09726884,   2.51545491,  -0.72721328},
 {    21,  -0.54242882,  -2.13719471,  -1.79430175},
 {   100,   0.43511009,   2.86426364,   0.04364074},
 {   128,   2.44512792,   1.46712252,   0.64692433},
 {    59,   2.16078081,  -1.45079232,   1.33645934},
 {    86,   2.39687545,   1.12879183,   1.26682556},
 {   127,   2.26373094,   0.58683966,  -1.79780511},
 };



struct TEST_DATASET1
{
	struct my_srba_options
	{
		typedef sensor_pose_on_robot_se3 sensor_pose_on_robot_t;
	};

	static basic_euclidean_dataset_entry_t * getData0(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#0.
		GT_pose = mrpt::poses::CPose3DQuat(0.063509,0.213511,0.308716, mrpt::math::CQuaternionDouble(0.950315,0.222352,0.172593,0.132939));

		N= sizeof(observations_0_displ)/sizeof(observations_0_displ[0]);
		return observations_0_displ;
	}

	static basic_euclidean_dataset_entry_t * getData1(size_t &N, mrpt::poses::CPose3DQuat &GT_pose)
	{
		// Observations for KF#10
		GT_pose = mrpt::poses::CPose3DQuat(1.299920,0.493723, 0.417527, mrpt::math::CQuaternionDouble(0.958629,0.237424,0.149111,0.049256));

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
