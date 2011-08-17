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
