/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers

#include <mrpt/config.h>
#include <mrpt/core/cpu.h>
#include <mrpt/core/format.h>

#if defined(_MSC_VER)
#include <intrin.h>  // __cpuidex
#endif

namespace mrpt::cpu::internal
{
CPU_analyzer& CPU_analyzer::Instance() noexcept
{
	static CPU_analyzer o;
	return o;
}

#if defined(_MSC_VER)
// MSVC version
void CPU_analyzer::detect_impl() noexcept
{
	// Based on: https://stackoverflow.com/a/7495023/1631514
#define cpuid(info, x) __cpuidex(info, x, 0)

	int info[4];
	cpuid(info, 0);
	int nIds = info[0];

	cpuid(info, 0x80000000);
	unsigned nExIds = info[0];

	//  Detect Features
	using namespace mrpt::cpu;

	if (nIds >= 0x00000001)
	{
		cpuid(info, 0x00000001);
		feat(feature::MMX) = !!(info[3] & (1 << 23));
		feat(feature::POPCNT) = !!(info[2] & (1 << 23));
		feat(feature::SSE) = !!(info[3] & (1 << 25));
		feat(feature::SSE2) = !!(info[3] & (1 << 26));
		feat(feature::SSE3) = !!(info[2] & (1 << 0));
		feat(feature::SSSE3) = !!(info[2] & (1 << 9));
		feat(feature::SSE4_1) = !!(info[2] & (1 << 19));
		feat(feature::SSE4_2) = !!(info[2] & (1 << 20));
		feat(feature::AVX) = !!(info[2] & (1 << 28));
	}
	if (nIds >= 0x00000007)
	{
		cpuid(info, 0x00000007);
		feat(feature::AVX2) = !!(info[1] & (1 << 5));
	}

	// Doubt: is this required?
	// auto xcrFeatureMask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
}
#else
// GCC/CLANG version
void CPU_analyzer::detect_impl() noexcept
{
	// __builtin_cpu_supports() checks for both: CPU and OS support

	using namespace mrpt::cpu;

	// These ones only for intel 64bit arch:
	// Some might be usable in 32bit builds, but they might require
	// additional checks (in MSVC) and it's not worth: why building for
	// 32bit nowadays?
#if defined(__x86_64__)
	feat(feature::MMX) = !!__builtin_cpu_supports("mmx");
	feat(feature::POPCNT) = !!__builtin_cpu_supports("popcnt");
	feat(feature::SSE) = !!__builtin_cpu_supports("sse");
	feat(feature::SSE2) = !!__builtin_cpu_supports("sse2");
	feat(feature::SSE3) = !!__builtin_cpu_supports("sse3");
	feat(feature::SSSE3) = !!__builtin_cpu_supports("ssse3");
	feat(feature::SSE4_1) = !!__builtin_cpu_supports("sse4.1");
	feat(feature::SSE4_2) = !!__builtin_cpu_supports("sse4.2");
	feat(feature::AVX) = __builtin_cpu_supports("avx");
	feat(feature::AVX2) = __builtin_cpu_supports("avx2");
#endif
}
#endif
}  // namespace mrpt::cpu::internal

std::string mrpt::cpu::features_as_string() noexcept
{
	std::string s;
	using namespace mrpt::cpu;
	const auto& o = internal::CPU_analyzer::Instance();

	s += mrpt::format("MMX:%i ", o.feat(feature::MMX) ? 1 : 0);
	s += mrpt::format("POPCNT:%i ", o.feat(feature::POPCNT) ? 1 : 0);
	s += mrpt::format("SSE:%i ", o.feat(feature::SSE) ? 1 : 0);
	s += mrpt::format("SSE2:%i ", o.feat(feature::SSE2) ? 1 : 0);
	s += mrpt::format("SSE3:%i ", o.feat(feature::SSE3) ? 1 : 0);
	s += mrpt::format("SSSE3:%i ", o.feat(feature::SSSE3) ? 1 : 0);
	s += mrpt::format("SSE4_1:%i ", o.feat(feature::SSE4_1) ? 1 : 0);
	s += mrpt::format("SSE4_2:%i ", o.feat(feature::SSE4_2) ? 1 : 0);
	s += mrpt::format("AVX:%i ", o.feat(feature::AVX) ? 1 : 0);
	s += mrpt::format("AVX2:%i ", o.feat(feature::AVX2) ? 1 : 0);

	return s;
}
