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
#ifndef CMappedImage_H
#define CMappedImage_H

#include <mrpt/utils/CImage.h>

namespace mrpt
{
	namespace utils
	{
		/** This class encapsulates a MRPT Image and allows the sampling of individual pixels with sub-pixel accuracy and with a change of coordinates (eg, meters).
		 *  Only work with graylevels (for convenience), so if a color image is passed it'll be passed first to grayscale.
		 *
		 * \sa CImage
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CMappedImage
		{
		protected:
			CImagePtr			m_img;
			double					m_x0,m_x1, m_y0, m_y1;
			double					m_pixel_size; //!< width * pixel_size = (x1-x0)
			TInterpolationMethod	m_method;

		public:
			/** Constructor: Must pass an image (as a smart pointer) and the coordinates of the border
			  * \param img The image. A copy of the smart pointer is kept internally to this object.
			  * \param x0 Coordinate X of the left side (default: 0)
			  * \param x1 Coordinate X of the right side (or -1 to IMAGE_WIDTH-1)
			  * \param y0 Coordinate Y of the top side (default: 0)
			  * \param y1 Coordinate Y of the bottom side (or -1 to IMAGE_HEIGHT-1)
			  * \param method The interpolation method: It can be imNEAREST, imBILINEAR or imBICUBIC.
			  */
			CMappedImage( CImagePtr img, double x0=0, double x1=-1, double y0=0, double y1=-1, TInterpolationMethod	method = IMG_INTERP_LINEAR );

			/** Changes the coordinates of the image (see constructor for the meaning) */
			void changeCoordinates(double x0, double x1, double y0, double y1);

			/** Returns the interpolated pixel at the coordinates (x,y), in the range [0,255] (grayscale) 
			  *  If the point is out of the image, 0 is returned.
			  */
			double getPixel(double x,double y ) const;


		}; // End of class


	} // end of namespace utils

} // end of namespace mrpt

#endif
