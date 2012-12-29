/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#include <mrpt/base.h>  // Precompiled headers



#include <MRPT/config.h>

#ifdef MRPT_OS_WINDOWS

#include <MRPT/UTILS/CEnhancedMetaFile.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CImage.h>

#include <windows.h>


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;


int CEnhancedMetaFile::LINUX_IMG_WIDTH = 800;
int CEnhancedMetaFile::LINUX_IMG_HEIGHT = 600;

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CEnhancedMetaFile::CEnhancedMetaFile(
	const std::string &targetFileName,
	int		scaleFactor ) :
		m_scale(scaleFactor),
		m_hFont(NULL)
{
	m_hdc = CreateEnhMetaFileA( NULL, targetFileName.c_str(), NULL, NULL );
	if (!m_hdc.get())
		THROW_EXCEPTION("Can't create EMF file!!!");
}

/*---------------------------------------------------------------
						Destructor
---------------------------------------------------------------*/
CEnhancedMetaFile::~CEnhancedMetaFile( )
{
	// Free objects:
	if (m_hFont.get())
	{
		DeleteObject(m_hFont.get());
		m_hFont = NULL;
	}

	// Finish EMF:
	DeleteEnhMetaFile( CloseEnhMetaFile( (HDC)m_hdc.get() ) );
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void  CEnhancedMetaFile::drawImage(
	int						x,
	int						y,
	const utils::CImage	&img
	)
{
	try
	{
		LPBITMAPINFO		pBmpInfo = (LPBITMAPINFO) new unsigned char[sizeof(BITMAPINFOHEADER) + (256 * sizeof(RGBQUAD))];
//		LPBITMAPINFO		pBmpInfo = (LPBITMAPINFO) new unsigned char[sizeof(BITMAPINFOHEADER) ];

		unsigned int		imgWidth = (unsigned int)img.getWidth();
		unsigned int		imgHeight = (unsigned int)img.getHeight();

		pBmpInfo->bmiHeader.biSize			= sizeof( BITMAPINFOHEADER );
		pBmpInfo->bmiHeader.biWidth			= imgWidth;
		pBmpInfo->bmiHeader.biHeight		= imgHeight;
		pBmpInfo->bmiHeader.biPlanes		= 1;
//		pBmpInfo->bmiHeader.biBitCount		= 24;
		pBmpInfo->bmiHeader.biBitCount		= 8;
		pBmpInfo->bmiHeader.biCompression	= BI_RGB;
		pBmpInfo->bmiHeader.biSizeImage		= 0;
		pBmpInfo->bmiHeader.biXPelsPerMeter	=
		pBmpInfo->bmiHeader.biYPelsPerMeter	= 0;
		pBmpInfo->bmiHeader.biClrUsed		= 0;
		pBmpInfo->bmiHeader.biClrImportant	= 0;

		// Palette
		for (unsigned char i=0; i<255; i++)
		{
			pBmpInfo->bmiColors[i].rgbRed      = i;
			pBmpInfo->bmiColors[i].rgbGreen    = i;
			pBmpInfo->bmiColors[i].rgbBlue     = i;
			pBmpInfo->bmiColors[i].rgbReserved = 0;
		}


//		unsigned int	lineBytes = 3*bmp.Width;
		unsigned int	lineBytes = imgWidth;
		if (lineBytes % 2) lineBytes++;
		if (lineBytes % 4) lineBytes+=2;

		BYTE			*ptrBits = new BYTE[lineBytes * imgHeight];

		for (unsigned int py=0;py<imgHeight;py++)
			for (unsigned int px=0;px<imgWidth;px++)
				ptrBits[(py*lineBytes+px)+0] = *img(px,py);

		HBITMAP	hBitmap = CreateDIBitmap(
			(HDC)m_hdc.get(),
			&pBmpInfo->bmiHeader,
			CBM_INIT,
			ptrBits,
			pBmpInfo,
			DIB_RGB_COLORS);

		ASSERT_(hBitmap!=NULL);

		BITMAP bm;
		GetObject(hBitmap,sizeof(bm),&bm);

		HDC hdcMem = CreateCompatibleDC( (HDC)m_hdc.get() );
		HBITMAP hbmT = (HBITMAP)SelectObject(hdcMem,hBitmap);

		BitBlt(
			(HDC)m_hdc.get(),
			x,
			y,
			(int)(m_scale*imgWidth),
			(int)(m_scale*imgHeight),
			hdcMem,
			0,
			0,
			SRCCOPY);

		SelectObject(hdcMem,hbmT);
		DeleteDC(hdcMem);

		// Free mem:
		// ---------------------------------------
		DeleteObject( hBitmap );
		delete[] pBmpInfo;
		delete[] ptrBits;
	}
	catch(...)
	{
		THROW_EXCEPTION("Unexpected runtime error!!");
	}
}

/*---------------------------------------------------------------
						drawBitmap
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
	x0*= m_scale;  y0*= m_scale;
	x1*= m_scale;  y1*= m_scale;

	HPEN	hPen = CreatePen(
		penStyle,
		width,
		(unsigned int)color );

	HPEN hOldPen = (HPEN) SelectObject( (HDC)m_hdc.get(), hPen );

	MoveToEx( (HDC)m_hdc.get(), x0,y0, NULL );
	LineTo( (HDC)m_hdc.get(), x1,y1);

	SelectObject( (HDC)m_hdc.get(), hOldPen  );
	DeleteObject( hPen );
}

/*---------------------------------------------------------------
						drawBitmap
---------------------------------------------------------------*/
void  CEnhancedMetaFile::textOut(
	int					x0,
	int					y0,
	const std::string	&str,
	const mrpt::utils::TColor color
	)
{
	x0*=m_scale; y0*=m_scale;

	::SetBkMode((HDC)m_hdc.get(), TRANSPARENT);
	::SetTextColor( (HDC)m_hdc.get(),(unsigned int)color);

	::TextOutA( (HDC)m_hdc.get(), x0,y0, str.c_str(), (int)str.size());
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
	HFONT		hFont,oldFont;
	LOGFONTA	lpf;

    lpf.lfHeight			= fontSize;
    lpf.lfWidth				= 0;
    lpf.lfEscapement		= 0;
    lpf.lfOrientation		= 0;
    lpf.lfWeight			= bold ? 700:400;
    lpf.lfItalic			= italic ? 1:0;
    lpf.lfUnderline			= 0;
    lpf.lfStrikeOut			= 0;
    lpf.lfCharSet			= DEFAULT_CHARSET;
    lpf.lfOutPrecision		= OUT_DEFAULT_PRECIS;
    lpf.lfClipPrecision		= CLIP_DEFAULT_PRECIS;
    lpf.lfQuality			= DEFAULT_QUALITY;
    lpf.lfPitchAndFamily	= DEFAULT_PITCH;
	os::strcpy(lpf.lfFaceName,LF_FACESIZE,fontName.c_str());

	hFont = ::CreateFontIndirectA( &lpf );

	oldFont = (HFONT)::SelectObject( (HDC)m_hdc.get(),hFont );

	if (oldFont)
		::DeleteObject(oldFont);
}


/*---------------------------------------------------------------
					selectTextFont
---------------------------------------------------------------*/
void  CEnhancedMetaFile::setPixel( int x, int y, size_t color)
{
	::SetPixel((HDC)m_hdc.get(),x*m_scale,y*m_scale,color);
}

/*---------------------------------------------------------------
						rectangle
---------------------------------------------------------------*/
void  CEnhancedMetaFile::rectangle(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	TColor			color,
	unsigned int	width)
{
	line(x0,y0,x1,y0,color,width);
	line(x1,y0,x1,y1,color,width);
	line(x1,y1,x0,y1,color,width);
	line(x0,y1,x0,y0,color,width);
}

#endif  // MRPT_OS_WINDOWS

