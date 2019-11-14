/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/common.h>
#include <array>
#include <string>

namespace mrpt::cpu
{
/** OS-portable set of CPU feature definitions, for usage in mrpt::cpu::supports
 * \ingroup mrpt_core_grp
 */
enum class feature : unsigned int
{
	MMX = 0,
	POPCNT,
	SSE,
	SSE2,
	SSE3,
	SSSE3,
	SSE4_1,
	SSE4_2,
	AVX,
	AVX2,
	// ---- end of list ----
	FEATURE_COUNT
};

namespace internal
{
/** Auxiliary class. Users should use mrpt::core::supports() instead.
 * \ingroup mrpt_core_grp
 */
class CPU_analyzer
{
   public:
	static CPU_analyzer& Instance() noexcept;

	std::array<
		bool, static_cast<std::size_t>(mrpt::cpu::feature::FEATURE_COUNT)>
		feat_detected;

	inline bool& feat(mrpt::cpu::feature f) noexcept
	{
		return feat_detected[static_cast<std::size_t>(f)];
	}
	inline const bool& feat(mrpt::cpu::feature f) const noexcept
	{
		return feat_detected[static_cast<std::size_t>(f)];
	}

   private:
	// Ctor: runs all the checks and fills in the vector of features:
	CPU_analyzer() noexcept
	{
		// Start with all falses:
		feat_detected.fill(false);
		detect_impl();
	}
	void detect_impl() noexcept;
};
}  // namespace internal

/** Returns true if CPU (and OS) supports the given CPU feature, *and* that
 * instruction set or feature was also enabled by the compiler flags used while
 * building MRPT.
 * \ingroup mrpt_core_grp
 */
inline bool supports(feature f) noexcept
{
	const auto& o = internal::CPU_analyzer::Instance();
	return o.feat(f);
}

/** Returns a string with detected features: "MMX:1 SSE2:0 etc."
 * \ingroup mrpt_core_grp
 */
std::string features_as_string() noexcept;

}  // namespace mrpt::cpu
