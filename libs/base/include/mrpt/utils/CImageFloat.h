/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef CImageFloat_H
#define CImageFloat_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CCanvas.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

namespace mrpt
{
	namespace utils
	{

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CImageFloat, mrpt::utils::CSerializable )

	/** In this class a grayscale image can be stored with float-type pixels.
	 *     I/O is supported as conversion to a byte-type pixels "CImage", and as binary dump using the
	 *       CSerializable interface(<< and >> operators), just as most objects in the MRPT library. This format
	 *			is not compatible with any standarized image format. <br>
	 * Additional notes: <br>
	 *		- Only the top-left coordinates origin format is supported.
	 *		- The format of the pixels is valid float values in the range [0,1]
	 *		- When assigning a CImage to a CImageFloat, the pixels are automatically converted from the range (0,255) --> (0,1)
	 *      - There is a "=" operator for converting between the classes "CImage" and "CImageFloat".
     *
	 *
	 * \sa CImage, CSerializable
	 */
	class BASE_IMPEXP CImageFloat : public mrpt::utils::CSerializable, public CCanvas
	{
		friend class CImage;

		DEFINE_SERIALIZABLE( CImageFloat )

	protected:
		/** Data members
		*/
		float			*m_img;

		/** The image size:
		  */
		size_t	m_width,m_height;

	public:
		/** Changes the value of the pixel (x,y).
		  *  Pixel coordinates starts at the left-top corner of the image, and start in (0,0).
		  *  The meaning of the parameter "color" depends on the implementation: it will usually
		  *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray level.
		  *  This method must support (x,y) values OUT of the actual image size without neither
		  *   raising exceptions, nor leading to memory access errors.
		  */
		void  setPixel(int x, int y, size_t color);

		/**  Resize the buffer "img" to accomodate a new image size.
		  */
		void  resize(
				size_t	width,
				size_t	height);

		/**  Resize the buffer "img" to accomodate a new image size.
		  *	Initialize all images pixels to zero.
		  *  by AJOGD @ JAN-2007
		  */
		void  setSize(
				size_t	width,
				size_t	height);

		/** Default constructor:
		 */
		CImageFloat(	size_t	width = 1,
							size_t	height = 1 );

		/** Copy constructor:
		 */
		CImageFloat( const CImageFloat &o );

		/** Copy constructor from a matrix with values in the range [0,1]:
		 */
		explicit CImageFloat( const math::CMatrixFloat &o );

		/** Copy constructor from a matrix with values in the range [0,1]:
		 */
		explicit CImageFloat( const math::CMatrixDouble &o );

		/** Copy constructor from a CImage.
		 */
		explicit CImageFloat( const CImage &o );

		/** Copy operator
		  */
		void  operator = (const CImageFloat& o);

		/** Copy operator
		  */
		void  operator = (const CImage& o);

		/** Copy operator from a float matrix. */
		void  operator = (const math::CMatrixFloat& o);

		/** Copy operator from a double matrix. */
		void  operator = (const math::CMatrixDouble& o);

		/** Destructor:
		 */
		virtual ~CImageFloat( );

		/** Returns the width of the image in pixels
		  */
		size_t  getWidth() const;

		/** Returns the height of the image in pixels
		  */
		size_t  getHeight() const;

		/** Returns a matrix CMatrix representation of the image.
		  *   The output matrix will contain at its (r,c) element the
		  *    pixel at the r'th row (y coordinate), and c'th column (x coordinate)
		  */
		void  getAsMatrix( math::CMatrixFloat &outMatrix ) const;

		/** Loads the image from a gray-scale image file.
  		  * See CImage::loadFromFile for supported formats
		 * \return False on any error
		  */
		bool  loadFromFile( const std::string& fileName  );

		/** Save the image to a file in a format given by the file extension.
		  * If "verticalFlip" is true, the image is inverted vertically
  		  * See CImage::saveToFile for supported formats
		 * \return False on any error
		  */
		bool  saveToFile(const std::string &fileName, bool verticalFlip = false) const;

		/** Save to a text file, loadable from matlab
		  */
		void  saveToTextFile(const std::string &fileName) const;

		/** Adjusts the range of the values in the image, such as the minimum and maximum values are the given ones.
		  */
		void  adjustImageRange(float min=0.0f, float max=1.0f);

		/** Returns a pointer to a given pixel.
		 *   The coordinate origin is pixel(0,0)=top-left corner of the image.
		 * \exception std::exception On pixel coordinates out of bounds
		 */
		float*  operator()(size_t col, size_t row) const;

		/** Point-wise product of CImageFloat images
		  * by FAMD, JAN-2007
		  */
		CImageFloat  operator*( const CImageFloat &im2 );
		/** Point-wise addition of CImageFloat images
		  * by FAMD, JAN-2007
  		  */
		CImageFloat  operator+( const CImageFloat &im2 );
		/** Point-wise substraction of CImageFloat images
		  * by FAMD, JAN-2007
		*/
		CImageFloat  operator-( const CImageFloat &im2 );
		/** Point-wise scalar power of CImageFloat images
		  * by FAMD, JAN-2007
		*/
		CImageFloat  operator^( int exp );

		/** Substitutes this image with a new one scaled down to half its original size.
		  */
		void  scaleHalf();


	}; // End of class

	typedef CImageFloat CMRPTImageFloat;	//!< Deprecated name.


	} // end of namespace utils
} // end of namespace mrpt

#endif
