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

#include <mrpt/img/TColor.h>
#include <mrpt/typemeta/TEnumType.h>

#include <tuple>

namespace mrpt::img
{
/** \addtogroup color_maps Color map functions (in #include
 * <mrpt/img/color_maps.h>)
 *  \ingroup mrpt_img_grp
 * @{ */

/** Transform HSV color components to RGB, all of them in the range [0,1]  \sa
 * rgb2hsv */
mrpt::img::TColorf hsv2rgb(float h, float s, float v);

/** Transform RGB color components to HSV, all of them in the range [0,1] \sa
 * hsv2rgb */
std::tuple<float, float, float> rgb2hsv(float r, float g, float b);

/** Different colormaps for use in mrpt::img::colormap() */
enum TColormap : int8_t
{
  /** Undefined colormap */
  cmNONE = -1,
  cmGRAYSCALE = 0,
  cmJET = 1,
  cmHOT = 2
};

/** Transform a float number in the range [0,1] into float RGB components (0,1).
 * Different colormaps are available.
 * \note The returned TColorf can be converted to TColor() with col.asTColor(); */
mrpt::img::TColorf colormap(const TColormap& color_map, float color_index);

/** Computes the RGB color components (range [0,1]) for the corresponding color
 * index in the range [0,1] using the MATLAB 'jet' colormap.  \sa colormap  */
mrpt::img::TColorf jet2rgb(float color_index);

/** Computes the RGB color components (range [0,1]) for the corresponding color
 * index in the range [0,1] using the MATLAB 'hot' colormap.  \sa colormap  */
mrpt::img::TColorf hot2rgb(float color_index);

/** @} */
}  // namespace mrpt::img
MRPT_ENUM_TYPE_BEGIN(mrpt::img::TColormap)
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmNONE);
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmGRAYSCALE);
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmJET);
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmHOT);
MRPT_ENUM_TYPE_END()
