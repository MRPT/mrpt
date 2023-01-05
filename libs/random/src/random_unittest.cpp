/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/random/random_shuffle.h>

TEST(Random, Randomize)
{
	using namespace mrpt::random;

	CRandomGenerator rnd;
	rnd.randomize(1);
	auto r1a = rnd.drawUniform32bit();
	rnd.randomize(2);
	auto r2a = rnd.drawUniform32bit();

	EXPECT_NE(r1a, r2a);

	rnd.randomize(1);
	auto r1abis = rnd.drawUniform32bit();
	EXPECT_EQ(r1a, r1abis);
}

TEST(Random, KnownSequence)
{
	const uint32_t seeds[5] = {
		1155500485U, 3258920289U, 2969816455U, 1886781181U, 2957178843U};

	const uint32_t psr_seq[5][30] = {
		{
			87159525U,	 981136246U,  3024579417U, 810904736U,	3227960653U,
			2925322908U, 3041022042U, 8514697U,	   238460451U,	3240846511U,
			77705137U,	 3667487648U, 2812201483U, 2852662203U, 3563562135U,
			4182703524U, 2338642573U, 2453315481U, 893072632U,	555674637U,
			1958390156U, 2936372696U, 2958209661U, 3206235417U, 1393170953U,
			3459948533U, 3978094773U, 187006958U,  1094993014U, 3697569806U,
		},
		{
			740306787U,	 1213665442U, 129332614U,  337857013U,	2827697054U,
			3921115624U, 3306026490U, 1982870663U, 3991370713U, 3031672994U,
			2187111251U, 2916846545U, 1508932967U, 4099179631U, 1795296805U,
			729143648U,	 3078055758U, 2112818007U, 2996444232U, 2352470411U,
			1889456200U, 823633826U,  4271544093U, 3164819009U, 3155209922U,
			934184739U,	 948314756U,  566002989U,  209716310U,	424306665U,
		},
		{
			4240204481U, 2332672160U, 2972009226U, 2500467979U, 2795915919U,
			772807544U,	 3550866598U, 2774481806U, 2630334347U, 145201545U,
			1244209075U, 3412246962U, 412965843U,  1254713838U, 2414279976U,
			1634686767U, 1198938400U, 2482335177U, 3662174704U, 2090231824U,
			73364990U,	 151986868U,  3009478209U, 4042788106U, 2315167490U,
			3883378205U, 1405261027U, 2984375936U, 4084849752U, 151767298U,
		},
		{
			2442517705U, 3154299563U, 627772564U,  609815136U,	80940975U,
			1175442950U, 1488732740U, 1806874014U, 1160095287U, 288113135U,
			750941403U,	 1902059558U, 2231364929U, 1702611100U, 4036287674U,
			1609309781U, 1569584333U, 4232782017U, 682338830U,	3379158924U,
			2048597387U, 249255365U,  217448606U,  3204242587U, 2391908062U,
			1116368730U, 72649517U,	  1674133514U, 1285581724U, 1072863427U,
		},
		{
			1465891278U, 2717027808U, 3786898353U, 1843368031U, 60506783U,
			2698460302U, 3794204567U, 735403158U,  2154018573U, 354675041U,
			955687956U,	 54804884U,	  3250611262U, 2708021793U, 3768186789U,
			3803268914U, 1902115381U, 1003259193U, 1253851151U, 2465171896U,
			464381829U,	 705115420U,  857237366U,  1403154329U, 2290045722U,
			3880844611U, 251734302U,  1989036956U, 1141049890U, 1462380353U,
		}};

	for (int seed = 0; seed < 5; seed++)
	{
		auto& rnd = mrpt::random::getRandomGenerator();
		mrpt::random::Randomize(seeds[seed]);
		for (int i = 0; i < 30; i++)
		{
			const auto v = rnd.drawUniform32bit();
			EXPECT_EQ(v, psr_seq[seed][i])
				<< "seed=" << seed << " i=" << i << "\n";
		}
	}
}

TEST(Random, drawGaussianMultivariateMany)
{
	using namespace mrpt::random;

	CRandomGenerator rnd;
	rnd.randomize(1);
	std::vector<std::vector<double>> samples;
	const size_t nSamples = 1000;
	mrpt::math::CMatrixDouble33 cov = mrpt::math::CMatrixDouble33::Zero();
	cov(0, 0) = 1.0;
	cov(1, 1) = cov(2, 2) = 2.0;

	rnd.drawGaussianMultivariateMany(samples, nSamples, cov);

	EXPECT_EQ(samples.size(), nSamples);
	EXPECT_EQ(samples.at(0).size(), static_cast<size_t>(cov.rows()));
}

TEST(Random, portable_uniform_distribution)
{
	mrpt::random::Generator_MT19937 rnd;

	const uint32_t seeds[5] = {0U, 1U, 3041022042U, 3206235417U, 2112818007U};

	const uint32_t psr_seq[5][30] = {
		{
			63, 28, 27, 95, 95, 62, 75, 64, 44, 57, 15, 90, 45, 93, 84,
			88, 8,	5,	11, 44, 31, 5,	73, 63, 43, 9,	49, 36, 94, 23,
		},
		{
			14, 52, 16, 57, 76, 37, 59, 18, 68, 53, 23, 50, 14, 11, 68,
			54, 35, 30, 34, 72, 34, 63, 8,	62, 35, 14, 48, 69, 30, 65,
		},
		{
			12, 52, 18, 43, 43, 21, 48, 36, 29, 47, 49, 20, 32, 13, 22,
			23, 32, 20, 45, 19, 46, 55, 42, 31, 49, 21, 28, 42, 59, 11,
		},
		{
			20, 24, 30, 35, 34, 33, 19, 23, 22, 38, 35, 34, 28, 27, 35,
			16, 15, 27, 19, 38, 26, 39, 26, 35, 31, 19, 24, 15, 31, 30,
		},
		{
			20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
			20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
		},
	};

	for (int seed = 0; seed < 5; seed++)
	{
		rnd.seed(seeds[seed]);
		for (int i = 0; i < 30; i++)
		{
			const auto v = mrpt::random::portable_uniform_distribution(
				rnd, seed * 5, 100 - 20 * seed);
			EXPECT_EQ(v, psr_seq[seed][i])
				<< "seed=" << seed << " i=" << i << "\n";
		}
	}
}

TEST(Random, shuffle)
{
	// Fixed, platform-independent RNG, so we have reproducible results below:
	mrpt::random::Generator_MT19937 rnd;
	rnd.seed(1);

	const std::vector<int> listOrg = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

	{
		auto list2 = listOrg;
		mrpt::random::shuffle(list2.begin(), list2.end(), rnd);
		EXPECT_EQ(list2, std::vector<int>({7, 2, 9, 8, 5, 6, 1, 4, 0, 3}));
	}

	{
		auto list2 = listOrg;
		mrpt::random::partial_shuffle(list2.begin(), list2.end(), rnd, 0);
		EXPECT_EQ(list2, listOrg);
	}

	{
		auto list2 = listOrg;
		mrpt::random::partial_shuffle(list2.begin(), list2.end(), rnd, 1);
		EXPECT_EQ(list2, std::vector<int>({6, 1, 2, 3, 4, 5, 0, 7, 8, 9}));
	}
	{
		auto list2 = listOrg;
		mrpt::random::partial_shuffle(list2.begin(), list2.end(), rnd, 10);
		EXPECT_EQ(list2, std::vector<int>({3, 5, 8, 0, 7, 1, 6, 2, 4, 9}));
	}
}
