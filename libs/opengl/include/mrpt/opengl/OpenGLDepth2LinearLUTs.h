/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <unordered_map>
#include <cstdlib>

namespace mrpt::opengl
{

/** Creates look-up-tables (LUTs) to map raw OpenGL depth values in the
 *  range [-1,1] to real, linear, depth values.
 *
 * \sa CFBORender
 * \ingroup mrpt_opengl_grp
 */
template <int DEPTH_LUT_NUM_BITS = 18>
class OpenGLDepth2LinearLUTs
{
   public:
	constexpr static std::size_t NUM_ENTRIES = 1 << DEPTH_LUT_NUM_BITS;

	static OpenGLDepth2LinearLUTs& Instance()
	{
		thread_local OpenGLDepth2LinearLUTs lut;
		return lut;
	}

	using lut_t = std::vector<float>;

	lut_t& lut_from_zn_zf(float zn, float zf)
	{
		const auto p = std::pair<float, float>(zn, zf);
		// reuse?
		if (auto it = m_pool.find(p); it != m_pool.end()) return it->second;

		// create new:
		auto& lut = m_pool[p];
		lut.resize(NUM_ENTRIES);

		const auto linearDepth = [zn, zf](float depthSample) -> float {
			if (depthSample == 1.0f) return 0.0f;  // no echo

			depthSample = 2.0f * depthSample - 1.0f;
			float zLinear =
				2.0f * zn * zf / (zf + zn - depthSample * (zf - zn));
			return zLinear;
		};

		for (size_t i = 0; i < NUM_ENTRIES; i++)
		{
			float f = -1.0f + 2.0f * static_cast<float>(i) / (NUM_ENTRIES - 1);
			lut.at(i) = linearDepth(f);
		}

		return lut;
	}

   private:
	struct MyHash
	{
		template <typename T>
		std::size_t operator()(const std::pair<T, T>& x) const
		{
			return std::hash<T>()(x.first) ^ std::hash<T>()(x.second);
		}
	};

	std::unordered_map<std::pair<float, float>, lut_t, MyHash> m_pool;
};

}
