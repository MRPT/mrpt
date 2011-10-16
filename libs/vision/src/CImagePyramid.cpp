/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision.h>  // Precompiled headers
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
