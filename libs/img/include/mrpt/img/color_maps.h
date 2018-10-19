/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/typemeta/TEnumType.h>

namespace mrpt::img
{
/** \addtogroup color_maps Color map functions (in #include
 * <mrpt/img/color_maps.h>)
 *  \ingroup mrpt_img_grp
 * @{ */

/** Transform HSV color components to RGB, all of them in the range [0,1]  \sa
 * rgb2hsv */
void hsv2rgb(float h, float s, float v, float& r, float& g, float& b);

/** Transform RGB color components to HSV, all of them in the range [0,1] \sa
 * hsv2rgb */
void rgb2hsv(float r, float g, float b, float& h, float& s, float& v);

/** Different colormaps for use in mrpt::img::colormap() */
enum TColormap
{
	cmNONE = -1, /** Undefined colormap [New in MRPT 2.0] */
	cmGRAYSCALE = 0,
	cmJET,
	/** [New in MRPT 1.5.0] */
	cmHOT
};

/** Transform a float number in the range [0,1] into RGB components. Different
 * colormaps are available. */
void colormap(
	const TColormap& color_map, const float color_index, float& r, float& g,
	float& b);

/** Computes the RGB color components (range [0,1]) for the corresponding color
 * index in the range [0,1] using the MATLAB 'jet' colormap.  \sa colormap  */
void jet2rgb(const float color_index, float& r, float& g, float& b);

/** Computes the RGB color components (range [0,1]) for the corresponding color
 * index in the range [0,1] using the MATLAB 'hot' colormap.  \sa colormap  */
void hot2rgb(const float color_index, float& r, float& g, float& b);

/** @} */
}  // namespace mrpt::img
MRPT_ENUM_TYPE_BEGIN(mrpt::img::TColormap)
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmNONE);
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmGRAYSCALE);
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmJET);
MRPT_FILL_ENUM_MEMBER(mrpt::img, cmHOT);
MRPT_ENUM_TYPE_END()
