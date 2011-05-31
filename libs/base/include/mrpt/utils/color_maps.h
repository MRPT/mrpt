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
