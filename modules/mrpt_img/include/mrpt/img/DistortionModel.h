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
  none = 0,       //!< No distortion applied
  plumb_bob,      //!< 5-param Brown-Conrady or Plumb-Bob pinhole distortion model
  kannala_brandt  //!< 4-parameter Kannala Brandt distortion model (fisheye)
};

}  // namespace mrpt::img

MRPT_ENUM_TYPE_BEGIN(mrpt::img::DistortionModel)
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, none);
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, plumb_bob);
MRPT_FILL_ENUM_MEMBER(mrpt::img::DistortionModel, kannala_brandt);
MRPT_ENUM_TYPE_END()
