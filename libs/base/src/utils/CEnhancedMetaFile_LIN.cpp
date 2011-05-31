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

#include <mrpt/base.h>  // Only for precomp. headers, include all libmrpt-core headers.



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
					selectTextFont
---------------------------------------------------------------*/
void  CEnhancedMetaFile::selectTextFont(
	const std::string  &fontName,
	int					fontSize,
	bool				bold,
	bool				italic )
{
    MRPT_TRY_START;

    ((CImage*)m_hdc.get())->selectTextFont(fontName);

    MRPT_TRY_END;
}


/*---------------------------------------------------------------
					selectTextFont
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

