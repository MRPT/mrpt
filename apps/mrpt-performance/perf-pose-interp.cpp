/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/datetime.h>

#include "common.h"

using mrpt::DEG2RAD;

// ------------------------------------------------------
//				Benchmark Pose Interpolation
// ------------------------------------------------------

// interpolator tests ======================
template <
	typename PATH_T, typename pose_t, bool INSERT_AT_END,
	bool BENCHMARK_INSERT_nQUERY>
double pose_interp_test(int a1, int a2)
{
	const long N = 400000;
	mrpt::system::CTicTac tictac;

	pose_t a = pose_t(
		mrpt::poses::CPose3D(1.0, 2.0, 0, DEG2RAD(10), .0, .0).asTPose());

	PATH_T pose_path;
	const auto t0 = mrpt::Clock::now();
	using namespace std::chrono_literals;
	const auto dt = 250ms;
	auto t = t0;

	std::vector<mrpt::Clock::duration> Ats(N);
	for (long i = 0; i < N; i++)
	{
		if (INSERT_AT_END)
		{
			Ats[i] = dt;
		}
		else
		{
			std::chrono::duration<double> randomDuration(
				mrpt::random::getRandomGenerator().drawUniform(-5.0, 5.0));
			Ats[i] = std::chrono::duration_cast<mrpt::Clock::duration>(
				randomDuration);
		}
	}

	if (BENCHMARK_INSERT_nQUERY)
	{
		tictac.Tic();
		for (long i = 0; i < N; i++)
		{
			pose_path.insert(t, a);
			t += Ats[i];  // negligible cost...
		}
		return tictac.Tac() / N;
	}
	else
	{
		for (long i = 0; i < N; i++)
		{
			pose_path.insert(t, a);
			t += 2 * Ats[i];
		}
		t = t0 + 4512ms;  // arbitrary period

		pose_t p;
		bool valid;
		tictac.Tic();
		for (long i = 0; i < N; i++)
		{
			pose_path.interpolate(t, p, valid);
			t += Ats[i];  // negligible cost...
		}
		dummy_do_nothing_with_string(
			mrpt::format("%s %s", p.asString().c_str(), valid ? "YES" : "NO"));
		return tictac.Tac() / N;
	}
}

// ------------------------------------------------------
// register_tests_pose_interp
// ------------------------------------------------------
void register_tests_pose_interp()
{
	mrpt::random::getRandomGenerator().randomize(1234);

	using mrpt::math::TPose2D;
	using mrpt::math::TPose3D;
	using namespace mrpt::poses;

	lstTests.emplace_back(
		"CPose3DInterpolator: CPose3D insert pose at end",
		&pose_interp_test<CPose3DInterpolator, CPose3D, true, true>);
	lstTests.emplace_back(
		"CPose3DInterpolator: CPose3D insert pose random",
		&pose_interp_test<CPose3DInterpolator, CPose3D, false, true>);
	lstTests.emplace_back(
		"CPose3DInterpolator: CPose3D query",
		&pose_interp_test<CPose3DInterpolator, CPose3D, true, false>);

	lstTests.emplace_back(
		"CPose3DInterpolator: TPose3D insert pose at end",
		&pose_interp_test<CPose3DInterpolator, TPose3D, true, true>);
	lstTests.emplace_back(
		"CPose3DInterpolator: TPose3D insert pose random",
		&pose_interp_test<CPose3DInterpolator, TPose3D, false, true>);
	lstTests.emplace_back(
		"CPose3DInterpolator: TPose3D query",
		&pose_interp_test<CPose3DInterpolator, TPose3D, true, false>);

	lstTests.emplace_back(
		"CPose2DInterpolator: TPose2D insert pose at end",
		&pose_interp_test<CPose2DInterpolator, TPose2D, true, true>);
	lstTests.emplace_back(
		"CPose2DInterpolator: TPose2D insert pose random",
		&pose_interp_test<CPose2DInterpolator, TPose2D, false, true>);
	lstTests.emplace_back(
		"CPose2DInterpolator: TPose2D query",
		&pose_interp_test<CPose2DInterpolator, TPose2D, true, false>);
}
