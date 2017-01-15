/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/config.h>

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <mrpt/utils/CEnhancedMetaFile.h>
#include <mrpt/utils/CImage.h>

using namespace mrpt;
using namespace mrpt::utils;


int CEnhancedMetaFile::LINUX_IMG_WIDTH = 800;
int CEnhancedMetaFile::LINUX_IMG_HEIGHT = 600;


/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CEnhancedMetaFile::CEnhancedMetaFile(
	const std::string &targetFileName,
	int		scaleFactor ) :
		m_scale(scaleFactor),
		m_hFont(NULL),
		m_targetFile(targetFileName)
{
    m_hdc = (void*) new CImage(CEnhancedMetaFile::LINUX_IMG_WIDTH, CEnhancedMetaFile::LINUX_IMG_HEIGHT );
    ((CImage*)m_hdc.get())->filledRectangle(0,0,CEnhancedMetaFile::LINUX_IMG_WIDTH-1, CEnhancedMetaFile::LINUX_IMG_HEIGHT-1, TColor(0,0,0) );
}

/*---------------------------------------------------------------
						Destructor
---------------------------------------------------------------*/
CEnhancedMetaFile::~CEnhancedMetaFile( )
{
    ((CImage*)m_hdc.get())->saveToFile(m_targetFile+".png");

	// Free objects:
	delete ((CImage*)m_hdc.get());
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void  CEnhancedMetaFile::drawImage(
	int						x,
	int						y,
	const CImage	&img
	)
{
    MRPT_TRY_START;

    ((CImage*)m_hdc.get())->drawImage(x,y,img);

    MRPT_TRY_END;
}

/*---------------------------------------------------------------
						line
---------------------------------------------------------------*/
void  CEnhancedMetaFile::line(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	const mrpt::utils::TColor color,
	unsigned int	width,
	TPenStyle		penStyle
	)
{
    MRPT_TRY_START;

    ((CImage*)m_hdc.get())->line(x0,y0,x1,y1,color,width,penStyle);

    MRPT_TRY_END;
}

/*---------------------------------------------------------------
						textOut
---------------------------------------------------------------*/
void  CEnhancedMetaFile::textOut(
	int					x0,
	int					y0,
	const std::string	&str,
	const mrpt::utils::TColor color
	)
{
    MRPT_TRY_START;

    ((CImage*)m_hdc.get())->textOut(x0,y0,str,color);

    MRPT_TRY_END;
}

/*---------------------------------------------------------------
					selectVectorTextFont
---------------------------------------------------------------*/
void  CEnhancedMetaFile::selectVectorTextFont(
	const std::string  &fontName,
	int					fontSize,
	bool				bold,
	bool				italic )
{
	MRPT_UNUSED_PARAM(fontSize); MRPT_UNUSED_PARAM(bold); MRPT_UNUSED_PARAM(italic);
    MRPT_TRY_START;

    ((CImage*)m_hdc.get())->selectTextFont(fontName);

    MRPT_TRY_END;
}


/*---------------------------------------------------------------
					setPixel
---------------------------------------------------------------*/
void  CEnhancedMetaFile::setPixel( int x, int y, size_t color)
{
    MRPT_TRY_START;

    ((CImage*)m_hdc.get())->setPixel(x,y, color);

    MRPT_TRY_END;
}

/*---------------------------------------------------------------
						rectangle
---------------------------------------------------------------*/
void  CEnhancedMetaFile::rectangle(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	const mrpt::utils::TColor color,
	unsigned int	width)
{
	line(x0,y0,x1,y0,color,width);
	line(x1,y0,x1,y1,color,width);
	line(x1,y1,x0,y1,color,width);
	line(x0,y1,x0,y0,color,width);
}

#endif

