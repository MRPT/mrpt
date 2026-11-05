/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/random/random_shuffle.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <vector>

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
  const uint32_t seeds[5] = {1155500485U, 3258920289U, 2969816455U, 1886781181U, 2957178843U};

  const uint32_t psr_seq[5][30] = {
      {
       87159525U, 981136246U, 3024579417U,  810904736U, 3227960653U,   2925322908U,
       3041022042U,    8514697U, 238460451U, 3240846511U,    77705137U, 3667487648U,
       2812201483U,  2852662203U, 3563562135U, 4182703524U,  2338642573U, 2453315481U,
       893072632U,  555674637U, 1958390156U,  2936372696U,  2958209661U,  3206235417U,
       1393170953U, 3459948533U, 3978094773U,    187006958U, 1094993014U, 3697569806U,
       },
      {
       740306787U, 1213665442U,   129332614U, 337857013U, 2827697054U,   3921115624U,
       3306026490U, 1982870663U,   3991370713U,  3031672994U,  2187111251U,   2916846545U,
       1508932967U,  4099179631U, 1795296805U,  729143648U, 3078055758U, 2112818007U,
       2996444232U,  2352470411U,  1889456200U,   823633826U, 4271544093U,  3164819009U,
       3155209922U,  934184739U, 948314756U,   566002989U,  209716310U, 424306665U,
       },
      {
       4240204481U,  2332672160U,  2972009226U, 2500467979U,  2795915919U,    772807544U,
       3550866598U, 2774481806U,   2630334347U,   145201545U, 1244209075U,   3412246962U,
       412965843U, 1254713838U, 2414279976U, 1634686767U,  1198938400U, 2482335177U,
       3662174704U,  2090231824U,    73364990U, 151986868U, 3009478209U,  4042788106U,
       2315167490U, 3883378205U, 1405261027U,   2984375936U,  4084849752U,  151767298U,
       },
      {
       2442517705U,  3154299563U,   627772564U, 609815136U,   80940975U, 1175442950U,
       1488732740U, 1806874014U,   1160095287U,   288113135U,  750941403U,  1902059558U,
       2231364929U,  1702611100U, 4036287674U, 1609309781U,  1569584333U, 4232782017U,
       682338830U, 3379158924U,  2048597387U,   249255365U,  217448606U, 3204242587U,
       2391908062U, 1116368730U,   72649517U, 1674133514U,  1285581724U, 1072863427U,
       },
      {
       1465891278U,  2717027808U,  3786898353U, 1843368031U,    60506783U, 2698460302U,
       3794204567U,  735403158U,  2154018573U,   354675041U,  955687956U,    54804884U,
       3250611262U,  2708021793U, 3768186789U, 3803268914U,  1902115381U, 1003259193U,
       1253851151U,  2465171896U,   464381829U,  705115420U,  857237366U, 1403154329U,
       2290045722U, 3880844611U,  251734302U,  1989036956U,  1141049890U, 1462380353U,
       }
  };

  for (int seed = 0; seed < 5; seed++)
  {
    auto& rnd = mrpt::random::getRandomGenerator();
    mrpt::random::Randomize(seeds[seed]);
    for (int i = 0; i < 30; i++)
    {
      const auto v = rnd.drawUniform32bit();
      EXPECT_EQ(v, psr_seq[seed][i]) << "seed=" << seed << " i=" << i << "\n";
    }
  }
}

TEST(Random, portable_uniform_distribution)
{
  mrpt::random::Generator_MT19937 rnd;

  const uint32_t seeds[5] = {0U, 1U, 3041022042U, 3206235417U, 2112818007U};

  // Expected values: min + ((hi << 32 | lo) % (max - min + 1)), with hi and lo
  // two consecutive outputs of MT19937 (see portableUniformDistributionFormula)
  const uint32_t psr_seq[5][30] = {
      {70, 85, 96, 28, 73,  2, 84, 50, 47, 39, 56, 91, 28, 65, 22,
       79, 24, 17, 97, 53, 94, 74, 33, 82, 100, 87, 90, 21, 13, 83},
      {40, 73, 38, 22, 17, 74,  40, 61, 78, 41,  54, 25, 49, 73, 47,
       37, 56, 46, 11, 63, 18, 20, 66, 42,  33, 50, 49, 64, 51, 61},
      {38, 38, 17, 15, 42, 45,  30, 31, 18, 14,  11, 15, 42, 56, 17,
       11, 22, 51, 10, 20, 29, 39, 22, 11,  59, 50, 54, 22, 39, 55},
      {17, 34, 31, 21, 29, 25,  21, 35, 24, 30,  32, 20, 22, 23, 22,
       32, 15, 30, 30, 15, 36, 21, 21, 37,  18, 30, 24, 16, 27, 26},
      {20, 20, 20, 20, 20, 20,  20, 20, 20, 20,  20, 20, 20, 20, 20,
       20, 20, 20, 20, 20, 20, 20, 20, 20,  20, 20, 20, 20, 20, 20},
  };

  for (uint64_t seed = 0; seed < 5; seed++)
  {
    rnd.seed(seeds[seed]);
    for (uint64_t i = 0; i < 30; i++)
    {
      const auto v = mrpt::random::portable_uniform_distribution(rnd, seed * 5, 100 - 20 * seed);
      EXPECT_EQ(v, psr_seq[seed][i]) << "seed=" << seed << " i=" << i << "\n";
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
    EXPECT_EQ(list2, std::vector<int>({0, 4, 7, 3, 6, 2, 8, 1, 5, 9}));
  }

  {
    auto list2 = listOrg;
    mrpt::random::partial_shuffle(list2.begin(), list2.end(), rnd, 0);
    EXPECT_EQ(list2, listOrg);
  }

  {
    auto list2 = listOrg;
    mrpt::random::partial_shuffle(list2.begin(), list2.end(), rnd, 1);
    EXPECT_EQ(list2, std::vector<int>({8, 1, 2, 3, 4, 5, 6, 7, 0, 9}));
  }
  {
    auto list2 = listOrg;
    mrpt::random::partial_shuffle(list2.begin(), list2.end(), rnd, 10);
    EXPECT_EQ(list2, std::vector<int>({3, 4, 6, 1, 0, 9, 5, 8, 7, 2}));
  }
}

/** Test for permuteVector (both versions) */
TEST(Random, PermuteVector)
{
  using namespace mrpt::random;
  CRandomGenerator rnd(123);  // Fixed seed for reproducibility

  std::vector<int> original(10);
  std::iota(original.begin(), original.end(), 0);  // 0, 1, 2, ..., 9

  // Test version 1: void permuteVector(const VEC& in_vector, VEC& out_result)
  std::vector<int> permuted1;
  rnd.permuteVector(original, permuted1);

  EXPECT_EQ(original.size(), permuted1.size());
  // Ensure all elements are still there
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(std::find(permuted1.begin(), permuted1.end(), i) != permuted1.end());
  }
  // With a fixed seed, we expect a change (statistically almost certain)
  EXPECT_NE(original, permuted1);

  // Test version 2: VEC permuteVector(const VEC& in_vector)
  auto permuted2 = rnd.permuteVector(original);
  EXPECT_EQ(original.size(), permuted2.size());
  EXPECT_NE(original, permuted2);
}

/** Test for drawUniformUnsignedIntRange */
TEST(Random, DrawUniformUnsignedIntRange)
{
  using namespace mrpt::random;
  CRandomGenerator rnd(456);

  uint32_t val;
  const uint32_t min_v = 10;
  const uint32_t max_v = 20;

  // Run multiple times to check range constraints
  for (int i = 0; i < 100; ++i)
  {
    rnd.drawUniformUnsignedIntRange(val, min_v, max_v);
    EXPECT_GE(val, min_v);
    EXPECT_LE(val, max_v);
  }
}

/** Test for drawUniformVector */
TEST(Random, DrawUniformVector)
{
  using namespace mrpt::random;
  CRandomGenerator rnd;

  std::vector<double> v(50);
  const double min_v = -5.0;
  const double max_v = 5.0;

  rnd.drawUniformVector(v, min_v, max_v);

  EXPECT_EQ(v.size(), 50U);
  for (const double val : v)
  {
    EXPECT_GE(val, min_v);
    EXPECT_LE(val, max_v);
  }
}

/** Test for Global/Static Helper Functions (vectorRandomUni) */
TEST(Random, GlobalHelpers)
{
  using namespace mrpt::random;

  // Test the global Randomize and vectorRandomUni
  Randomize(789);
  std::vector<float> v(10);
  vectorRandomUni(v, 0.0f, 1.0f);

  for (const float val : v)
  {
    EXPECT_GE(val, 0.0f);
    EXPECT_LE(val, 1.0f);
  }
}

TEST(Random, MT19937MatchesStdMt19937)
{
  for (const uint32_t seed : {0U, 1U, 5489U, 1234U, 4294967295U})
  {
    mrpt::random::Generator_MT19937 g;
    g.seed(seed);
    std::mt19937 ref(seed);
    for (int i = 0; i < 2000; i++)
    {
      ASSERT_EQ(g(), static_cast<uint32_t>(ref())) << "seed=" << seed << " i=" << i;
    }
  }
}

TEST(Random, portableUniformDistributionFormula)
{
  // Independent evaluation of the documented formula on the reference engine:
  mrpt::random::Generator_MT19937 g;
  g.seed(5);
  std::mt19937 ref(5);
  for (int i = 0; i < 200; i++)
  {
    const uint64_t hi = static_cast<uint32_t>(ref());
    const uint64_t lo = static_cast<uint32_t>(ref());
    const uint64_t expected = 7 + (((hi << 32) | lo) % (1000 - 7 + 1));
    EXPECT_EQ(mrpt::random::portable_uniform_distribution(g, 7, 1000), expected);
  }
  // Full 64-bit range: no division by zero
  const auto v =
      mrpt::random::portable_uniform_distribution(g, 0, std::numeric_limits<uint64_t>::max());
  (void)v;
}

TEST(Random, portableUniformDistributionInclusiveRange)
{
  mrpt::random::Generator_MT19937 g;
  g.seed(1);
  std::map<uint64_t, int> cnt;
  for (int i = 0; i < 3000; i++)
  {
    const auto v = mrpt::random::portable_uniform_distribution(g, 0, 2);
    ASSERT_LE(v, 2U);
    cnt[v]++;
  }
  ASSERT_EQ(cnt.size(), 3U);
  for (const auto& kv : cnt)
  {
    EXPECT_NEAR(kv.second, 1000, 120) << "value=" << kv.first;
  }
}

TEST(Random, shuffleIsUniformOverAllPermutations)
{
  mrpt::random::Generator_MT19937 g;
  g.seed(3);
  std::map<std::vector<int>, int> cnt;
  int firstStays = 0;
  const int N = 6000;
  for (int t = 0; t < N; t++)
  {
    std::vector<int> v = {0, 1, 2};
    mrpt::random::shuffle(v.begin(), v.end(), g);
    cnt[v]++;
    if (v[0] == 0)
    {
      firstStays++;
    }
  }
  EXPECT_EQ(cnt.size(), 6U);
  for (const auto& kv : cnt)
  {
    EXPECT_NEAR(kv.second, 1000, 150);
  }
  EXPECT_NEAR(firstStays, 2000, 200);
}

TEST(Random, partialShuffleIsUniform)
{
  // The first element after a partial shuffle of N=1 must be uniform over
  // the whole sequence.
  mrpt::random::Generator_MT19937 g;
  g.seed(11);
  std::map<int, int> cnt;
  for (int t = 0; t < 4000; t++)
  {
    std::vector<int> v = {0, 1, 2, 3};
    mrpt::random::partial_shuffle(v.begin(), v.end(), g, 1);
    cnt[v[0]]++;
  }
  ASSERT_EQ(cnt.size(), 4U);
  for (const auto& kv : cnt)
  {
    EXPECT_NEAR(kv.second, 1000, 150) << "value=" << kv.first;
  }
}

TEST(Random, permuteVectorReproducibleWithSeed)
{
  std::vector<int> v(20);
  std::iota(v.begin(), v.end(), 0);
  mrpt::random::CRandomGenerator a(42);
  mrpt::random::CRandomGenerator b(42);
  EXPECT_EQ(a.permuteVector(v), b.permuteVector(v));

  std::vector<int> o1;
  std::vector<int> o2;
  mrpt::random::getRandomGenerator().randomize(7);
  mrpt::random::randomPermutation(v, o1);
  mrpt::random::getRandomGenerator().randomize(7);
  mrpt::random::randomPermutation(v, o2);
  EXPECT_EQ(o1, o2);

  std::sort(o1.begin(), o1.end());
  EXPECT_EQ(o1, v);
}

TEST(Random, GaussianSequenceReproducibleAfterReseed)
{
  mrpt::random::CRandomGenerator a(5);
  (void)a.drawGaussian1D_normalized();  // may leave a cached sample
  a.randomize(1234);
  mrpt::random::CRandomGenerator b(1234);
  for (int i = 0; i < 4; i++)
  {
    EXPECT_EQ(a.drawGaussian1D_normalized(), b.drawGaussian1D_normalized()) << "i=" << i;
  }
}

TEST(Random, drawUniformMoments)
{
  mrpt::random::CRandomGenerator rng(1234);
  const int N = 200000;
  const double a = -3.0;
  const double b = 5.0;
  double s = 0;
  double s2 = 0;
  for (int i = 0; i < N; i++)
  {
    const double x = rng.drawUniform(a, b);
    ASSERT_GE(x, a);
    ASSERT_LE(x, b);
    s += x;
    s2 += x * x;
  }
  const double m = s / N;
  const double var = s2 / N - m * m;
  // 6 standard errors
  EXPECT_NEAR(m, 0.5 * (a + b), 6 * (b - a) / std::sqrt(12.0 * N));
  EXPECT_NEAR(var, (b - a) * (b - a) / 12.0, 0.03);
}

TEST(Random, drawGaussian1DMoments)
{
  mrpt::random::CRandomGenerator rng(4321);
  const int N = 200000;
  double s = 0;
  double s2 = 0;
  int within1 = 0;
  for (int i = 0; i < N; i++)
  {
    const double x = rng.drawGaussian1D(2.0, 3.0);
    s += x;
    s2 += x * x;
    if (std::abs(x - 2.0) < 3.0)
    {
      within1++;
    }
  }
  const double m = s / N;
  const double sd = std::sqrt(s2 / N - m * m);
  EXPECT_NEAR(m, 2.0, 6 * 3.0 / std::sqrt(N));
  EXPECT_NEAR(sd, 3.0, 0.03);
  // P(|x-mu|<sigma) = 0.682689492137
  EXPECT_NEAR(static_cast<double>(within1) / N, 0.682689492137, 0.006);
}

TEST(Random, drawUniformUnsignedIntRangeIsUniformAndInclusive)
{
  mrpt::random::CRandomGenerator rng(77);
  std::map<uint32_t, int> cnt;
  const int N = 11000;
  for (int i = 0; i < N; i++)
  {
    uint32_t v = 0;
    rng.drawUniformUnsignedIntRange(v, 10U, 20U);
    ASSERT_GE(v, 10U);
    ASSERT_LE(v, 20U);
    cnt[v]++;
  }
  ASSERT_EQ(cnt.size(), 11U);
  for (const auto& kv : cnt)
  {
    EXPECT_NEAR(kv.second, 1000, 150) << "value=" << kv.first;
  }
}

TEST(Random, drawUniformUnsignedIntRangeFullRange)
{
  mrpt::random::CRandomGenerator rng(1);
  uint32_t v32 = 0;
  rng.drawUniformUnsignedIntRange(v32, 0U, std::numeric_limits<uint32_t>::max());
  uint64_t v64 = 0;
  rng.drawUniformUnsignedIntRange(
      v64, static_cast<uint64_t>(0), std::numeric_limits<uint64_t>::max());
  // Two different full-range draws are (almost surely) not both zero:
  EXPECT_TRUE(v32 != 0 || v64 != 0);
}
