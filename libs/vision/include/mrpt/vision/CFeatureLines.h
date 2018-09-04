/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */
#pragma once

#include <mrpt/img/CImage.h>
#include <vector>
#include <array>

namespace mrpt
{
namespace vision
{
/**  This class wraps different line detectors and descriptors from OpenCV.
 *
 *  \ingroup mrpt_vision_grp
 */
class CFeatureLines
{
   public:
	void extractLines(
		const mrpt::img::CImage& image,
		std::vector<std::array<int, 4>>& segments, size_t threshold,
		const bool display = false);

};  // end of class

}  // namespace vision
}  // namespace mrpt
