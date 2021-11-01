/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
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
	None = 0,  //!< No distortion applied
	BrownConrady,  //!<  Brown-Conrady or Plumb-Bob pinhole distortion model
	KannalaBrandt  //!< Four parameter Kannala Brandt distortion model
};

}  // namespace mrpt::img

MRPT_ENUM_TYPE_BEGIN(mrpt::img::DistortionModel)
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, None);
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, BrownConrady);
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, KannalaBrandt);
MRPT_ENUM_TYPE_END()
