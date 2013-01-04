/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef color_maps_H
#define color_maps_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** Transform HSV color components to RGB, all of them in the range [0,1]
		  * \sa rgb2hsv
		  */
		void BASE_IMPEXP hsv2rgb(
			float	h,
			float	s,
			float	v,
			float	&r,
			float	&g,
			float	&b);

		/** Transform RGB color components to HSV, all of them in the range [0,1]
		  * \sa hsv2rgb
		  */
		void BASE_IMPEXP rgb2hsv(
			float	r,
			float	g,
			float	b,
			float	&h,
			float	&s,
			float	&v );

		/** Different colormaps
		  * \sa mrpt::vision::colormap
		  */
		enum TColormap
		{
			cmGRAYSCALE = 0,
			cmJET
		};

		/** Transform a float number in the range [0,1] into RGB components. Different colormaps are available.
		  */
		void BASE_IMPEXP colormap(
			const TColormap &color_map,
			const float	color_index,
			float	&r,
			float	&g,
			float	&b);

		/** Computes the RGB color components (range [0,1]) for the corresponding color index in the range [0,1] using the MATLAB 'jet' colormap.
		  * \sa colormap
		  */
		void BASE_IMPEXP jet2rgb(
			const float	color_index,
			float	&r,
			float	&g,
			float	&b);
	}
}


#endif
