/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mrpt::img
{
/** Enum for different camera distortion models.
 *
 * \sa TCamera
 * \ingroup mrpt_img_grp
 */
enum class DistortionModel : uint8_t
{
	none = 0,  //!< No distortion applied
	plumb_bob,	//!< 5-param Brown-Conrady or Plumb-Bob pinhole distortion model
	kannala_brandt	//!< 4-parameter Kannala Brandt distortion model (fisheye)
};

}  // namespace mrpt::img

MRPT_ENUM_TYPE_BEGIN(mrpt::img::DistortionModel)
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, none);
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, plumb_bob);
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, kannala_brandt);
MRPT_ENUM_TYPE_END()
