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
					selectVectorTextFont
---------------------------------------------------------------*/
void  CEnhancedMetaFile::selectVectorTextFont(
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

