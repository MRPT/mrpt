/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CMappedImage.h>
#include <mrpt/utils/round.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;


/*---------------------------------------------------------------
                Constructor
 ---------------------------------------------------------------*/
CMappedImage::CMappedImage( CImagePtr img, double x0, double x1, double y0, double y1, TInterpolationMethod	method  ) :
	m_img( img ),
	m_x0 (x0),
	m_x1 (x1),
	m_y0 (y0),
	m_y1 (y1),
	m_pixel_size(0),
	m_method( method )
{
	m_img->grayscale();
	if (m_img->isColor())
	{
		CImage *new_img = new CImage();
		m_img->grayscale(*new_img);
		m_img = CImagePtr( new_img );
	}
	changeCoordinates(x0,x1,y0,y1);
}

/*---------------------------------------------------------------
                changeCoordinates
 ---------------------------------------------------------------*/
void CMappedImage::changeCoordinates(double x0, double x1, double y0, double y1)
{
	MRPT_START
	ASSERT_(x0!=x1);
	ASSERT_(y0!=y1);

	m_x0 =x0;	m_x1 =x1;
	m_y0 =y0;	m_y1 =y1;

	if (y1<0 || x1<0)
	{
		m_x1 = m_img->getWidth()-1;
		m_y1 = m_img->getHeight()-1;
	}

	ASSERT_( m_img->getWidth()>0 && m_img->getHeight() );

	m_pixel_size = (m_x1-m_x0) / m_img->getWidth();

	MRPT_END
}

/*---------------------------------------------------------------
                getPixel
 ---------------------------------------------------------------*/
double CMappedImage::getPixel(double x,double y ) const
{
	// Image size:
	const size_t W = m_img->getWidth();
	const size_t H = m_img->getHeight();

	// the sub-pixel pixel coordinates:
	const double px = (x-m_x0)/m_pixel_size;
	const double py = (y-m_y0)/m_pixel_size;

	if (px<0 || py<0 || px>W || py>H) { return 0; }	// Out of image

	switch (m_method)
	{
	case IMG_INTERP_NN:
		{
			// The closest pixel:
			const int px0 = mrpt::utils::round(px);
			const int py0 = mrpt::utils::round(py);
			return static_cast<double>(*m_img->get_unsafe(px0, py0));
		}
		break;
	case IMG_INTERP_LINEAR:
		{
			// See: http://en.wikipedia.org/wiki/Bilinear_interpolation

			// The four pixels around:
			const int px0 = (int)floor(px);
			const int px1 = (int)ceil(px);
			const int py0 = (int)floor(py);
			const int py1 = (int)ceil(py);

			const double P11 = static_cast<double>(*m_img->get_unsafe(px0, py0));
			const double P12 = static_cast<double>(*m_img->get_unsafe(px0, py1));
			const double P21 = static_cast<double>(*m_img->get_unsafe(px1, py0));
			const double P22 = static_cast<double>(*m_img->get_unsafe(px1, py1));

			const double R1 = P11*(px1-px) /* /(px1-px0)*/ + P21*(px-px0) /* /(px1-px0) */;
			const double R2 = P12*(px1-px) /* /(px1-px0)*/ + P22*(px-px0) /* /(px1-px0) */;

			return R1 * (py1-py) + R2 * (py-py0);
		}
		break;

	case IMG_INTERP_CUBIC:
		{
			THROW_EXCEPTION("TO DO!");
		}
		break;

	case IMG_INTERP_AREA:
	default:
		THROW_EXCEPTION("The selected interpolation method is not supported in this method.");
	};

}

