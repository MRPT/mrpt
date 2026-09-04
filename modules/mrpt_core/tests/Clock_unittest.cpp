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
#include <mrpt/core/Clock.h>
#include <mrpt/core/config.h>

#include <chrono>
#include <cmath>
#include <thread>

namespace
{
void test_delay()
{
  const double t0 = mrpt::Clock::nowDouble();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const double t1 = mrpt::Clock::nowDouble();

  EXPECT_GT(t1 - t0, 0.008);  // ideally, near 0.010
  EXPECT_LT(t1 - t0, 5.0);    // just detect it's not a crazy number
}
}  // namespace

TEST(clock, delay_Realtime)
{
  // Default:
  test_delay();

  // Monotonic:
  mrpt::Clock::setActiveClock(mrpt::Clock::Source::Monotonic);
  test_delay();

#if 0  // non-repetitive results
	// mono->rt offset:
	const uint64_t d1 = mrpt::Clock::getMonotonicToRealtimeOffset();
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	mrpt::Clock::resetMonotonicToRealTimeEpoch();
	const uint64_t d2 = mrpt::Clock::getMonotonicToRealtimeOffset();
	EXPECT_GT(d1, d2);
#endif

  // Realtime:
  mrpt::Clock::setActiveClock(mrpt::Clock::Source::Realtime);
  test_delay();
}

TEST(clock, changeSource)
{
  const double t0 = mrpt::Clock::nowDouble();
  mrpt::Clock::setActiveClock(mrpt::Clock::Source::Monotonic);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const double t1 = mrpt::Clock::nowDouble();
  mrpt::Clock::setActiveClock(mrpt::Clock::Source::Realtime);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const double t2 = mrpt::Clock::nowDouble();

  EXPECT_GT(t1 - t0, 0.008);  // ideally, near 0.010
  EXPECT_LT(t1 - t0, 5.0);    // just detect it's not a crazy number

  EXPECT_GT(t2 - t1, 0.008);  // ideally, near 0.010
  EXPECT_LT(t2 - t1, 5.0);    // just detect it's not a crazy number
}

TEST(clock, checkSynchEpoch)
{
  for (int i = 0; i < 20; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const int64_t err = mrpt::Clock::resetMonotonicToRealTimeEpoch();

    // it should be a really small number in a regular computer,
    // but we set the threshold much higher due to spurious errors
    // when running unit tests in VMs (build farms)
#if MRPT_IN_EMSCRIPTEN
    const int64_t errLimit = 1000 * 1000;  // We are running on Javascript!
#else
    // normally much smaller, but for busy build servers
    const int64_t errLimit = static_cast<int64_t>(90 * 1000);
#endif

    EXPECT_LT(std::abs(err), errLimit);
  }
}

TEST(clock, fromDouble_toDouble_roundtrip)
{
  // Round-trip a range of UNIX timestamps through fromDouble()/toDouble().
  // The 1e-7 s resolution of Clock::duration bounds the achievable accuracy.
  const double timestamps[] = {
      0.0, 1e-3, 1.0, 100.0, 1300000000.123, 1700000000.5,
  };
  for (const double t : timestamps)
  {
    const double back = mrpt::Clock::toDouble(mrpt::Clock::fromDouble(t));
    EXPECT_NEAR(back, t, 1e-6) << "t=" << t;
  }
}

TEST(clock, fromDouble_negativeAndNearEpoch)
{
  // Regression test for an undefined-behavior bug: fromDouble() used to cast a
  // potentially-negative double directly to uint64_t. That cast wraps on
  // x86-64 but saturates to 0 on AArch64, so e.g. fromDouble(-1e-3) and
  // fromDouble(-2e-3) collapsed to the *same* tick on ARM while staying
  // distinct on x86. Such tiny negative timestamps appear when callers subtract
  // a small offset from a timestamp whose UNIX time is ~0 (datasets with
  // zeroed/relative stamps). Ordering and spacing must be preserved on every
  // platform.
  const auto tm2 = mrpt::Clock::fromDouble(-2e-3);
  const auto tm1 = mrpt::Clock::fromDouble(-1e-3);
  const auto t0 = mrpt::Clock::fromDouble(0.0);
  const auto tp1 = mrpt::Clock::fromDouble(+1e-3);

  // Strictly monotonic in the same order as the input doubles:
  EXPECT_LT(tm2.time_since_epoch().count(), tm1.time_since_epoch().count());
  EXPECT_LT(tm1.time_since_epoch().count(), t0.time_since_epoch().count());
  EXPECT_LT(t0.time_since_epoch().count(), tp1.time_since_epoch().count());

  // Each 1 ms step must be exactly 10000 ticks (1 tick = 100 ns):
  EXPECT_EQ(tm1.time_since_epoch().count() - tm2.time_since_epoch().count(), INT64_C(10000));
  EXPECT_EQ(t0.time_since_epoch().count() - tm1.time_since_epoch().count(), INT64_C(10000));

  // And the differences survive the conversion back to seconds:
  EXPECT_NEAR(mrpt::Clock::toDouble(tm1) - mrpt::Clock::toDouble(tm2), 1e-3, 1e-9);
}

TEST(clock, toDouble_preEpoch)
{
  // Regression test for an unsigned-wraparound bug: toDouble() used to
  // compute `t.time_since_epoch().count() - UINT64_C(116444736) *
  // UINT64_C(1000000000)` where both operands got promoted to uint64_t, even
  // though `count()` is a *signed* int64_t. For any tick count before the
  // 1970-01-01 UNIX epoch, the subtraction wrapped around and toDouble()
  // returned ~+1.8e12 instead of a small negative number. This single-value
  // check (as opposed to a difference of two toDouble() calls) is essential:
  // a difference cancels the 2^64 wraparound and would pass even with the
  // bug present.
  const auto tm2ms = mrpt::Clock::fromDouble(-2e-3);
  const double back = mrpt::Clock::toDouble(tm2ms);

  EXPECT_NEAR(back, -2e-3, 1e-6);
  EXPECT_LT(back, 0.0);
  EXPECT_LT(std::abs(back), 1.0);  // must NOT be ~1.8e12
}

TEST(clock, toDouble_monotonicAcrossEpoch)
{
  // toDouble() must be strictly increasing across the 1601->1970 epoch
  // boundary, i.e. no wraparound discontinuity for pre-epoch instants.
  const double doubles[] = {-2e-3, -1e-3, 0.0, 1e-3, 1.0, 100.0};
  double prev = mrpt::Clock::toDouble(mrpt::Clock::fromDouble(doubles[0]));
  for (size_t i = 1; i < sizeof(doubles) / sizeof(doubles[0]); i++)
  {
    const double cur = mrpt::Clock::toDouble(mrpt::Clock::fromDouble(doubles[i]));
    EXPECT_GT(cur, prev) << "i=" << i;
    prev = cur;
  }
}

TEST(clock, fromDouble_toDouble_roundtrip_negativeZeroPositive)
{
  // Round trip for negative, zero, and positive UNIX timestamps.
  const double timestamps[] = {-100.0, -1.0, -1e-3, 0.0, 1e-3, 1.0, 100.0};
  for (const double t : timestamps)
  {
    const double back = mrpt::Clock::toDouble(mrpt::Clock::fromDouble(t));
    EXPECT_NEAR(back, t, 1e-6) << "t=" << t;
  }
}

TEST(clock, toDouble_subMicrosecondAccuracy)
{
  // Regression test for a precision loss introduced while fixing the
  // pre-epoch wraparound above: converting `count()` to double *before*
  // subtracting the 1601->1970 offset. A raw tick count for a present-day
  // instant is ~1.3e17, more than an order of magnitude past 2^53, so that
  // conversion alone quantizes the result to ~1.6 us -- enough to perturb
  // per-scan timing in sensor pipelines that round-trip stamps through
  // doubles. Doing the shift in the integer domain first keeps the value
  // exactly representable.
  constexpr int64_t UNIX_EPOCH_OFFSET = INT64_C(116444736) * INT64_C(1000000000);

  // A real-world present-day stamp, plus neighbours one tick apart:
  const int64_t unixTicks = INT64_C(17319341181624082);

  for (int64_t k = 0; k < 64; k++)
  {
    const int64_t ticks = unixTicks + k;
    const auto tp = mrpt::Clock::time_point(mrpt::Clock::duration(ticks + UNIX_EPOCH_OFFSET));

    // Reference value of the same quantity, computed without the
    // intermediate rounding that the defect introduced:
    const long double exact = static_cast<long double>(ticks) / 10000000.0L;
    const long double err = std::abs(static_cast<long double>(mrpt::Clock::toDouble(tp)) - exact);

    // The result's own ULP at ~1.7e9 s is 2.4e-7 s, so the bound cannot be
    // tighter than that: on targets where `long double` is just a `double`
    // (arm64 macOS, MSVC) the reference itself is rounded, so the two can
    // legitimately differ by one full ULP. The defect was ~4x above this.
    EXPECT_LT(err, 3e-7L) << "k=" << k;
  }
}

TEST(clock, toDouble_distinguishesAdjacentTicks)
{
  // Stamps 1 us apart (10 ticks) must stay ordered and 1 us apart after
  // conversion. The bound cannot be tighter than the representation itself:
  // a double holding a present-day UNIX time has an ULP of ~2.4e-7 s, so
  // "1 us apart" is only ever recoverable to within one of those steps. What
  // the defect broke was coarser than that -- neighbouring stamps drifted by
  // several ULPs, or collapsed onto the same double entirely.
  constexpr int64_t UNIX_EPOCH_OFFSET = INT64_C(116444736) * INT64_C(1000000000);
  const int64_t unixTicks = INT64_C(17319341181624082);

  for (int64_t k = 0; k < 32; k++)
  {
    const auto a =
        mrpt::Clock::time_point(mrpt::Clock::duration(unixTicks + k * 10 + UNIX_EPOCH_OFFSET));
    const auto b = mrpt::Clock::time_point(
        mrpt::Clock::duration(unixTicks + (k + 1) * 10 + UNIX_EPOCH_OFFSET));

    const double ta = mrpt::Clock::toDouble(a);
    const double tb = mrpt::Clock::toDouble(b);

    EXPECT_GT(tb, ta) << "k=" << k;                 // never collapse
    EXPECT_NEAR(tb - ta, 1e-6, 3e-7) << "k=" << k;  // within one ULP of 1 us
  }
}

TEST(clock, simulatedTime)
{
  const auto t0 = mrpt::Clock::now();
  const auto prevSrc = mrpt::Clock::getActiveClock();
  // Enable simulated time:
  mrpt::Clock::setActiveClock(mrpt::Clock::Simulated);

  // Check that time is stopped:
  mrpt::Clock::setSimulatedTime(t0);
  const auto t1 = mrpt::Clock::now();
  EXPECT_EQ(t0, t1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto t2 = mrpt::Clock::now();
  EXPECT_EQ(t0, t2);

  // Check time moving forward:
  auto tset2 = t0 + std::chrono::milliseconds(1000);
  mrpt::Clock::setSimulatedTime(tset2);
  const auto t3 = mrpt::Clock::now();
  EXPECT_EQ(tset2, t3);

  // Restore
  mrpt::Clock::setActiveClock(prevSrc);
}
