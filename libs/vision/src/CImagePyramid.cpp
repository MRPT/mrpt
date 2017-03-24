/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers
#include <mrpt/vision/CImagePyramid.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::vision;

CImagePyramid::CImagePyramid()
{
}

CImagePyramid::~CImagePyramid()
{
	// Nothing especial to do, mem. is automatically freed.
}

// Template that generalizes the two user entry-points below:
template <bool FASTLOAD>
void buildPyramid_templ(
	CImagePyramid &obj,
	mrpt::utils::CImage &img,
	const size_t nOctaves,
	const bool smooth_halves,
	const bool convert_grayscale)
{
	ASSERT_ABOVE_(nOctaves,0)

	//TImageSize  img_size = img.getSize();
	obj.images.resize(nOctaves);

	// First octave: Just copy the image:
	if (convert_grayscale && img.isColor())
	{
		// In this case we have to convert to grayscale, so FASTLOAD doesn't really matter:
		img.grayscale(obj.images[0]);
	}
	else
	{
		// No need to convert to grayscale OR image already is grayscale:
		if (FASTLOAD)
		     obj.images[0].copyFastFrom(img);  // Fast copy -> "move", destroying source.
		else obj.images[0] = img;  // Normal copy
	}

	// Rest of octaves, if any:
	for (size_t o=1;o<nOctaves;o++)
	{
		if (smooth_halves)
		     obj.images[o-1].scaleHalfSmooth(obj.images[o]);
		else obj.images[o-1].scaleHalf(obj.images[o]);
	}
}

void CImagePyramid::buildPyramid(
	const mrpt::utils::CImage &img,
	const size_t nOctaves,
	const bool smooth_halves,
	const bool convert_grayscale)
{
	buildPyramid_templ<false>(*this,*const_cast<mrpt::utils::CImage*>(&img), nOctaves,smooth_halves,convert_grayscale);
}

void CImagePyramid::buildPyramidFast(
	mrpt::utils::CImage &img,
	const size_t nOctaves,
	const bool smooth_halves,
	const bool convert_grayscale)
{
	buildPyramid_templ<true>(*this,img,nOctaves,smooth_halves,convert_grayscale);
}
