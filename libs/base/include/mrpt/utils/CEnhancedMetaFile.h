/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CEnhancedMetaFile_H
#define  CEnhancedMetaFile_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CCanvas.h>
#include <mrpt/utils/safe_pointers.h>

namespace mrpt
{
namespace utils
{
	/** This class represents a Windows Enhanced Meta File (EMF) for generating and saving graphics.
	  *  If used under Linux, a ".png", non-vectorial, file will be generated instead.
	  * \ingroup mrpt_base_grp
	  */
	class BASE_IMPEXP CEnhancedMetaFile : public CCanvas
	{
	private:
		void_ptr_noncopy 	m_hdc;
		int		    		m_scale;
		void_ptr_noncopy 	m_hFont;
		std::string 		m_targetFile;

	public:
		static int LINUX_IMG_WIDTH;		//!< In Linux, the size of the bitmap image that emulates the EMF (Default:800)
		static int LINUX_IMG_HEIGHT;	//!< In Linux, the size of the bitmap image that emulates the EMF (Default:600)


		/** Constructor
		  *  \param targetFileName This file will be created and the EMF saved there.
		  *  \param scaleFactor All coordinates in draw commands will be internally multiplied by this scale, to provide a way of obtaining "subpixel" drawing.
		  */
		CEnhancedMetaFile(
			const std::string &targetFileName,
			int		scaleFactor = 1);

		/** Destructor */
		virtual ~CEnhancedMetaFile(  );

		/** Changes the value of the pixel (x,y).
		  *  Pixel coordinates starts at the left-top corner of the image, and start in (0,0).
		  *  The meaning of the parameter "color" depends on the implementation: it will usually
		  *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray level.
		  *  This method must support (x,y) values OUT of the actual image size without neither
		  *   raising exceptions, nor leading to memory access errors.
		  */
		void  setPixel( int x, int y, size_t color) MRPT_OVERRIDE;

		/** Returns the width of the image in pixels (this currently has no applicability for a EMF file...) */
		size_t  getWidth() const MRPT_OVERRIDE { return 640; }

		/** Returns the height of the image in pixels (this currently has no applicability for a EMF file...) */
		size_t getHeight() const MRPT_OVERRIDE {return 480;}

		/** Draws an image as a bitmap at a given position.
		  * \param x0 The top-left corner x coordinates on this canvas where the image is to be drawn
		  * \param y0 The top-left corner y coordinates on this canvas where the image is to be drawn
		  * \param img The image to be drawn in this canvas
		  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
		  */
		void  drawImage(int x, int y, const utils::CImage	&img ) MRPT_OVERRIDE;

		/** Draws a line.
		  * \param x0 The starting point x coordinate
		  * \param y0 The starting point y coordinate
		  * \param x1 The end point x coordinate
		  * \param y1 The end point y coordinate
		  * \param color The color of the line
		  * \param width The desired width of the line (this is IGNORED in this virtual class)
		  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
		  */
		void  line(
			int x0, int y0,
			int x1, int y1,
			const mrpt::utils::TColor color,
			unsigned int	width = 1,
			TPenStyle		penStyle = psSolid) MRPT_OVERRIDE;

		/** Places a text label.
		  * \param x0 The x coordinates
		  * \param y0 The y coordinates
		  * \param str The string to put
		  * \param color The text color
		  * \param fontSize The font size, in "points"
		  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
		  * \sa rectangle
		  */
		void  textOut(
			int x0, int y0,
			const std::string	&str,
			const mrpt::utils::TColor color
			) MRPT_OVERRIDE;

		/** Select the current font used when drawing text.
		  * \param fontName The face name of a font (e.g. "Arial","System",...)
		  * \param fontSize The size of the font in pts.
		  * \param bold Whether the font is bold
		  * \param italic Whether the font is italic
		  * \sa textOut, CCanvas::selectTextFont
		  */
		virtual void  selectVectorTextFont(
			const std::string  &fontName,
			int					fontSize,
			bool				bold = false,
			bool				italic = false );

		/** Draws an image as a bitmap at a given position, with some custom scale and rotation changes.
		  * \param x0 The top-left corner x coordinates on this canvas where the image is to be drawn
		  * \param y0 The top-left corner y coordinates on this canvas where the image is to be drawn
		  * \param rotation The rotation in radians, positive values being anti-clockwise direction, 0 is the normal position.
		  * \param scale The scale factor, e.g. 2 means twice the original size.
		  * \param img The image to be drawn in this canvas
		  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
		  */
		void drawImage(
			int x, int y,
			const utils::CImage	&img,
			float rotation,
			float scale ) MRPT_OVERRIDE
		{
			CCanvas::drawImage(x,y,img,rotation,scale);
		}

		/** Draws a rectangle (an empty rectangle, without filling)
		  * \param x0 The top-left x coordinate
		  * \param y0 The top-left y coordinate
		  * \param x1 The right-bottom x coordinate
		  * \param y1 The right-bottom y coordinate
		  * \param color The color of the line
		  * \param width The desired width of the line.
		  * \sa filledRectangle
		  */
		virtual void  rectangle(
			int				x0,
			int				y0,
			int				x1,
			int				y1,
			const mrpt::utils::TColor color,
			unsigned int	width = 1 );

		/** Draws an ellipse representing a given confidence interval of a 2D Gaussian distribution.
		  * \param mean_x The x coordinate of the center point of the ellipse.
		  * \param mean_y The y coordinate of the center point of the ellipse.
		  * \param cov2D A 2x2 covariance matrix.
		  * \param confIntervalStds How many "sigmas" for the confidence level (i.e. 2->95%, 3=99.97%,...)
		  * \param color The color of the ellipse
		  * \param width The desired width of the line (this is IGNORED in this virtual class)
		  * \param nEllipsePoints The number of points to generate to approximate the ellipse shape.
		  * \exception std::exception On an invalid matrix.
		  */
		template <class T>
		void  ellipseGaussian(
			math::CMatrixTemplateNumeric<T>	*cov2D,
			T							mean_x,
			T							mean_y,
			float						confIntervalStds = 2,
			const mrpt::utils::TColor 	&color = mrpt::utils::TColor(255,255,255),
			unsigned int				width = 1,
			int							nEllipsePoints = 20
			)
		{
			MRPT_START
			int								x1=0,y1=0,x2=0,y2=0;
			double							ang;
			math::CMatrixTemplateNumeric<T>		eigVal,eigVec;
			int								i;

			// Compute the eigen-vectors & values:
			cov2D->eigenVectors(eigVec,eigVal);

			eigVal.Sqrt();
			math::CMatrixTemplateNumeric<T>		M( eigVal * (~eigVec) );

			// Compute the points of the 2D ellipse:
			for (i=0,ang=0;i<nEllipsePoints;i++,ang+= (M_2PI/(nEllipsePoints-1)))
			{
				float	ccos = cos(ang);
				float	ssin = sin(ang);

				x2 = round( mean_x + confIntervalStds * (ccos * M(0,0) + ssin * M(1,0)) );
				y2 = round( mean_y + confIntervalStds * (ccos * M(0,1) + ssin * M(1,1)) );

				if (i>0)
					line( x1, y1,x2, y2,color,width );

				x1 = x2;
				y1 = y2;
			} // end for points on ellipse

			MRPT_END_WITH_CLEAN_UP( \
				std::cout << "Covariance matrix leading to error is:" << std::endl << *cov2D << std::endl; \
				);
		}
	}; // End of class def.
	} // End of namespace
} // end of namespace
#endif
