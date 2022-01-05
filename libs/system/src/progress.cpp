/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>
#include <mrpt/system/progress.h>

#include <array>

std::string mrpt::system::progress(
	const double progressRatio0to1, const std::size_t barLength,
	bool encloseInSquareBrackets)
{
	// Inspired in:
	// https://github.com/verigak/progress/blob/master/progress/bar.py
	// (BSD License)

#ifdef _WIN32
	const std::array<std::string, 3> phases = {" ", "▌", "█"};
#else
	const std::array<std::string, 9> phases = {
		//
		" ", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"};
#endif

	MRPT_START

	ASSERT_GE_(progressRatio0to1, 0.0);
	ASSERT_LE_(progressRatio0to1, 1.0);
	ASSERT_GE_(barLength, 1);

	std::string s;
	if (encloseInSquareBrackets) s = "[";

	const size_t nphases = phases.size();
	const double filled_len = barLength * progressRatio0to1;
	// Number of full chars
	const size_t nfull = static_cast<size_t>(filled_len);
	// Phase of last char
	const size_t phase = static_cast<size_t>((filled_len - nfull) * nphases);
	int nempty = int(barLength - nfull) - 1;  // Number of empty chars

	for (size_t i = 0; i < nfull; i++)
		s += phases.back();

	if (phase > 0) s += phases.at(phase);
	else
	{
		nempty++;
	}

	for (int i = 0; i < nempty; i++)
		s += phases.front();

	if (encloseInSquareBrackets) s += "]";

	return s;
	MRPT_END
}
