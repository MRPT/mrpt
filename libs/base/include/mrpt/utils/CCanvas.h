/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CCanvas_H
#define CCanvas_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/math/eigen_frwds.h>

namespace mrpt
{
	namespace utils
	{
		class CImage;

		/** This virtual class defines the interface of any object accepting drawing primitives on it.
		 *
		 *  A number of text fonts can be selected with CCanvas::selectTextFont(). These are the
		 *   implemented font names:
		 *
		 *  - "6x13"
		 *  - "6x13B" (bold)
		 *  - "6x13O" (italic)
		 *  - "9x15"
		 *  - "9x15B" (bold)
		 *  - "10x20"
		 *  - "18x18ja" (Japanese, UNICODE character values)
		 *
		 *  For an example of each font check the <a href="http://www.mrpt.org/Implemented_2D_Fonts">corresponding wiki page</a>.
		 *
		 * \sa CImage
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CCanvas
		{
		protected:
			std::string     m_selectedFont; //!< The selected font name.

			const uint32_t *m_selectedFontBitmaps; //!< Direct access to character bitmaps.

		public:

			CCanvas();

			/** Definition of pen styles
			  */
			enum TPenStyle
			{
				psSolid = 0,
				psDash,			/* -------  */
				psDot,			/* .......  */
				psDashDot,		/* _._._._  */
				psDashDotDot	/* _.._.._  */
			};

			/** Dummy virtual destructor:
			  */
			virtual ~CCanvas()
			{
			}

			/** Changes the value of the pixel (x,y).
			  *  Pixel coordinates starts at the left-top corner of the image, and start in (0,0).
			  *  The meaning of the parameter "color" depends on the implementation: it will usually
			  *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray level.
			  *
			  *  You can also use a TColor() type as input and it will be automatically converted to size_t.
			  *
			  *  This method must support (x,y) values OUT of the actual image size without neither
			  *   raising exceptions, nor leading to memory access errors.
			  *
			  */
			virtual void  setPixel( int x, int y, size_t color) =0;

			/** Returns the width of the image in pixels
			  */
			virtual size_t  getWidth() const = 0;

			/** Returns the height of the image in pixels
			  */
			virtual size_t  getHeight() const = 0;

			/** Draws a line.
			  * \param x0 The starting point x coordinate
			  * \param y0 The starting point y coordinate
			  * \param x1 The end point x coordinate
			  * \param y1 The end point y coordinate
			  * \param color The color of the line
			  * \param width The desired width of the line (this is IGNORED in this virtual class)
			  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
			  */
			virtual void  line(
				int				x0,
				int				y0,
				int				x1,
				int				y1,
				const mrpt::utils::TColor color,
				unsigned int	width = 1,
				TPenStyle		penStyle = psSolid);

			/** Draws a rectangle (an empty rectangle, without filling)
			  * \param x0 The top-left x coordinate
			  * \param y0 The top-left y coordinate
			  * \param x1 The right-bottom x coordinate
			  * \param y1 The right-bottom y coordinate
			  * \param color The color of the line
			  * \param width The desired width of the line.
			  * \sa filledRectangle
			  */
			void  rectangle(
				int				x0,
				int				y0,
				int				x1,
				int				y1,
				const mrpt::utils::TColor	color,
				unsigned int	width = 1 );

	/*****************************************************AJOGD***************************************************/
			/** Draws a triangle
			  * \param x0 The triangle center x coordinate
			  * \param y0 The triangle center y coordinate
			  * \param size The size of the triangle
			  * \param color The color of the line
			  *	\param inferior The position of the triangle
			  * \param width The desired width of the line.
			  * \sa triangle
			  */
			void  triangle(
				int				x0,
				int				y0,
				int				size,
				const mrpt::utils::TColor	color,
				bool			inferior = true,
				unsigned int	width = 1 );
	/************************************************************************************************************/

			/** Draws a filled rectangle.
			  * \param x0 The top-left x coordinate
			  * \param y0 The top-left y coordinate
			  * \param x1 The right-bottom x coordinate
			  * \param y1 The right-bottom y coordinate
			  * \param color The color of the rectangle fill
			  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
			  * \sa rectangle
			  */
			virtual void  filledRectangle(
				int				x0,
				int				y0,
				int				x1,
				int				y1,
				const mrpt::utils::TColor	color
				);

			/** Renders 2D text using bitmap fonts.
			  * \param x0 The x coordinates
			  * \param y0 The y coordinates
			  * \param str The string to put. If using UNICODE characters, use UTF-8 encoding.
			  * \param color The text color
			  *
			  * \sa selectTextFont
			  */
			virtual void  textOut(
				int					x0,
				int					y0,
				const std::string	&str,
				const mrpt::utils::TColor	color
				);

			/** Select the current font used when drawing text.
			  * \param fontName The name of the font
			  *
			  *  Valid font names:
			  *  - 5x7
			  *  - 6x13
			  *  - 6x13B
			  *  - 6x13O
			  *  - 9x15   (Default at start-up)
			  *  - 9x15B
			  *  - 10x20
			  *  - 18x18ja (Asian characters for UTF-8 strings - Only available if MRPT is built with MRPT_HAS_ASIAN_FONTS = true)
			  *
			  *   <img src="sample_textFonts.png" >
			  *
			  * \sa textOut, The example in <a href="http://www.mrpt.org/Implemented_2D_Fonts">this page</a>.
			  */
			virtual void  selectTextFont( const std::string  &fontName );

			/** Draws an image as a bitmap at a given position.
			  * \param x0 The top-left corner x coordinates on this canvas where the image is to be drawn
			  * \param y0 The top-left corner y coordinates on this canvas where the image is to be drawn
			  * \param img The image to be drawn in this canvas
			  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
			  */
			virtual void  drawImage(
				int						x,
				int						y,
				const utils::CImage	&img );

			/** Draw a cross.
			* \param x0 The point x coordinate
			* \param y0 The point y coordinate
			* \param color The color of the cross
			* \param size The size of the cross
			* \param type The cross type. It could be: 'x', '+' or ':'(like '+' but clear at the center dot)
			* \param width The desired width of the cross (this is IGNORED yet)
			*/
			void  cross (int x0,int y0, const mrpt::utils::TColor	color,char type, unsigned int size=5, unsigned int width = 1);

			/** Draws an image as a bitmap at a given position, with some custom scale and rotation changes.
			  * \param x0 The top-left corner x coordinates on this canvas where the image is to be drawn
			  * \param y0 The top-left corner y coordinates on this canvas where the image is to be drawn
			  * \param rotation The rotation in radians, positive values being anti-clockwise direction, 0 is the normal position.
			  * \param scale The scale factor, e.g. 2 means twice the original size.
			  * \param img The image to be drawn in this canvas
			  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
			  */
			virtual void  drawImage(
				int						x,
				int						y,
				const utils::CImage	&img,
				float					rotation,
				float					scale );

			/** Draws a circle of a given radius.
			  * \param x The center - x coordinate in pixels.
			  * \param y The center - y coordinate in pixels.
			  * \param radius The radius - in pixels.
			  * \param color The color of the circle.
			  * \param width The desired width of the line (this is IGNORED in this virtual class)
			  */
			virtual void  drawCircle(
				int		x,
				int		y,
				int		radius,
				const mrpt::utils::TColor	&color = mrpt::utils::TColor(255,255,255),
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
			template <class MATRIX2X2>
			void  ellipseGaussian(
				const MATRIX2X2 			*cov2D,
				const double				mean_x,
				const double				mean_y,
				double						confIntervalStds = 2,
				const mrpt::utils::TColor 	&color = mrpt::utils::TColor(255,255,255),
				unsigned int				width = 1,
				int							nEllipsePoints = 20
				)
			{
				MRPT_START
				int				x1=0,y1=0,x2=0,y2=0;
				double			ang;
				MATRIX2X2		eigVal,eigVec;
				int				i;

				// Compute the eigen-vectors & values:
				cov2D->eigenVectors(eigVec,eigVal);

				eigVal = eigVal.array().sqrt().matrix();
				MATRIX2X2	M;
				M.multiply_ABt(eigVal, eigVec);

				// Compute the points of the 2D ellipse:
				for (i=0,ang=0;i<nEllipsePoints;i++,ang+= (M_2PI/(nEllipsePoints-1)))
				{
					double ccos = cos(ang);
					double ssin = sin(ang);

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

			/** Draws a set of marks onto the image, given a generic container of entities having just "x" and "y" fields.
			 *  The class of FEATURELIST can be, for example, std::vector<mrpt::math::TPoint2D>, std::vector<TPixelCoordsf> or mrpt::vision::CFeatureList
			 * \sa drawFeatures
			 */
			template <class FEATURELIST>
			void  drawFeaturesSimple( const FEATURELIST &list, const TColor &color = TColor::red, const int cross_size = 5 )
			{
				for(size_t i=0;i<list.size(); ++i )
				{
					const int x = round( list.getFeatureX(i) );
					const int y = round( list.getFeatureY(i) );
					this->cross( x,y, color, '+', cross_size );
				}
			}

			/** Draws a set of marks (or scaled circles for features with scale) onto the image, given a generic container of features.
			 *  The class of FEATURELIST can be:
			 *    - mrpt::vision::CFeatureList
			 *    - mrpt::vision::TSimpleFeatureList
			 *
			 * \sa drawFeaturesSimple
			 */
			template <class FEATURELIST>
			void  drawFeatures( const FEATURELIST &list, const TColor &color = TColor::red, const bool showIDs = false, const bool showResponse = false  )
			{
				for(size_t i=0;i<list.size(); ++i )
				{
					const int x = round( list.getFeatureX(i) );
					const int y = round( list.getFeatureY(i) );
					this->cross(x,y, color, '+' );
					if( showIDs ) this->textOut(x,y, format("%u", static_cast<unsigned int>(list.getFeatureID(i))), TColor::red );
					if (showResponse) this->textOut( x,y+10, format("R:%u", static_cast<unsigned int>(list.getFeatureResponse(i))), TColor::red );
					if( ! list.isPointFeature(i) ) this->drawCircle(x,y, list.getScale(i), TColor::red );
				}
			}
		}; // End of class

	} // end of namespace utils

} // end of namespace mrpt

#endif
