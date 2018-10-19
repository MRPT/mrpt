/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TColor.h>
#include <utility>
#include <set>
#include <cstdlib>
#include <iostream>

namespace mrpt::img
{
// TODO - finish this.
/**\brief Manage R, G, B color triads and ask class instance of the next
 * unique RGB combination.
 */
struct TColorManager
{
   public:
	/**\brief Constructor */
	TColorManager(bool use_standard_colors_first = true);
	/**\brief Destructor */
	~TColorManager();
	/**\brief Get the next RGB triad in TColorf form.
	 *
	 * Method automatically advances the current counters for RGB.
	 */
	mrpt::img::TColorf getNextTColorf();
	/**\brief Get the next RGB triad in TColor form.
	 *
	 * Method automatically advances the current counters for RGB.
	 */
	mrpt::img::TColor getNextTColor();
	mrpt::img::TColor curr_color;
	std::set<mrpt::img::TColor> used_colors;
	/** Indicates if the standard colors are to be returned first.
	 */
	bool use_standard_colors_first;
	/** Indicates if the standard colors have already been used.
	 *
	 * This is a cached version of the value returned from the
	 * checkStdColorsUsed method.
	 *
	 * \note Standard colors are: red, green, blue
	 *
	 * \sa checkStdColorsUsed
	 */
	bool have_used_standard_colors;
	/**\brief Check if the standard colors have already been used.
	 *
	 * \ret True if they have indeed been used.
	 */
	bool checkStdColorsUsed();

	uint8_t color_step;
	const uint8_t color_thresh{255};

	bool have_exceeded_colors;

   private:
	// color triad with which to advance the current TColor instance
	mrpt::img::TColor color_step_triad;

	/**\brief Reset all class properties to their default values
	 *
	 * Method called in the constructor of the class
	 */
	void reset();
	void advanceRGBCounters();
	/**\brief Mark the given color as used. */
	void markColorAsUsed(mrpt::img::TColor color);
};
}  // namespace mrpt::img
