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

#ifndef __mrpt_vision_image_pyramid_H
#define __mrpt_vision_image_pyramid_H

#include <mrpt/utils/CImage.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** Holds and builds a pyramid of images: starting with an image at full resolution (octave=1), it builds
		  *  a number of half-resolution images: octave=2 at 1/2 , octave=3 at 1/2^2, octave=N at 1/2^(N-1).
		  *
		  *  Color (RGB) or grayscale pyramids can be built from color input images; only grayscale pyramids can be built from
		  *   grayscale images.
		  *
		  *  The algorithm to halve the images can be either a 1:2 decimation or a smooth filter (arithmetic mean of every 4 pixels).
		  *
		  *  Pyramids are built by invoking the method \a buildPyramid() or \a buildPyramidFast()
		  *
		  * Example of usage:
		  * \code
		  *   CImagePyramid  pyr;
		  *
		  *   CImage img = ...
		  *
		  *   pyr.buildPyramid(
		  *      img,
		  *      4,    // num. of octaves
		  *      true  // smooth
		  *      );
		  *
		  *   pyr.images[0].saveToFile("pyr0.jpg");
		  *   pyr.images[1].saveToFile("pyr1.jpg");
		  *   ...
		  * \endcode
		  *
		  *  \note Both converting to grayscale and building the octave images have SSE2-optimized implementations (if available).
		  *
		  * \sa mrpt::utils::CImage
		  * \ingroup mrpt_vision_grp 
		  */
		class VISION_IMPEXP CImagePyramid
		{
		public:
			CImagePyramid();   //!< Default constructor, does nothing
			~CImagePyramid();  //!< Destructor, frees the stored images.

			/** Fills the vector \a images with the different octaves built from the input image.
			  *  \param[in] img The input image. Can be either color or grayscale.
			  *  \param[in] nOctaves Number of octaves to build. 1 means just the original image, 2 means the original plus the 1/2 image, etc.
			  *  \param[in] smooth_halves If true, use an arithmetic mean of every 2x2 pixel block when downsampling.
			  *  \param[in] convert_grayscale If true, the pyramid is built in grayscale even for color input images.
			  * \sa buildPyramidFast
			  */
			void buildPyramid(const mrpt::utils::CImage &img, const size_t nOctaves, const bool smooth_halves = true, const bool convert_grayscale = false );

			/**  Exactly like \a buildPyramid(), but if the input image has not to be converted from RGB to grayscale, the image data buffer is *reutilized*
			  *   for the 1st octave in \a images[0], emptying the input image.
			  * \sa buildPyramid
			  */
			void buildPyramidFast(mrpt::utils::CImage &img, const size_t nOctaves, const bool smooth_halves = true, const bool convert_grayscale = false );

			/** The individual images:
			  *  - images[0]: 1st octave (full-size)
			  *  - images[1]: 2nd octave (1/2 size)
			  *  - images[2]: 3rd octave (1/4 size)
			  */
			std::vector<mrpt::utils::CImage>  images;
		};

	}
}

#endif
