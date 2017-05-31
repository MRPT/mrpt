/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#ifndef TCOLORMANAGER_H
#define TCOLORMANAGER_H

#include <mrpt/utils/TColor.h>
#include <mrpt/utils/mrpt_macros.h>
#include <utility>
#include <set>
#include <cstdlib>
#include <iostream>

namespace mrpt { namespace utils {

// TODO - finish this.
/**\brief Manage R, G, B color triads and ask class instance of the next
 * unique RGB combination.
 */
struct TColorManager {
	public:

		/**\brief Constructor */
		TColorManager(
				bool use_standard_colors_first=true);
		/**\brief Destructor */
		~TColorManager();
		/**\brief Get the next RGB triad in TColorf form.
		 *
		 * Method automatically advances the current counters for RGB.
		 */
		mrpt::utils::TColorf getNextTColorf();
		/**\brief Get the next RGB triad in TColor form.
		 *
		 * Method automatically advances the current counters for RGB.
		 */
		mrpt::utils::TColor getNextTColor();
		mrpt::utils::TColor curr_color;
		std::set<mrpt::utils::TColor> used_colors;
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
		const uint8_t color_thresh;

		bool have_exceeded_colors;

	private:
		// color triad with which to advance the current TColor instance
		mrpt::utils::TColor color_step_triad;

		/**\brief Reset all class properties to their default values
		 *
		 * Method called in the constructor of the class
		 */
		void reset();
		void advanceRGBCounters();
		/**\brief Mark the given color as used. */
		void markColorAsUsed(mrpt::utils::TColor color);
};

} } // end of namespaces

#endif /* end of include guard: TCOLORMANAGER_H */
