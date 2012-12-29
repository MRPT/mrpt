/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
