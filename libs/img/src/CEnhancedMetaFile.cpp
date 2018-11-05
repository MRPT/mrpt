/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

#include <mrpt/img/CEnhancedMetaFile.h>
#include <mrpt/system/os.h>
#include <mrpt/img/CImage.h>

#include <mrpt/config.h>
#ifdef _WIN32
#include <windows.h>
#endif

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::system;

static int LINUX_IMG_WIDTH_value = 800;
static int LINUX_IMG_HEIGHT_value = 600;

void CEnhancedMetaFile::LINUX_IMG_WIDTH(int value)
{
	LINUX_IMG_WIDTH_value = value;
}
int CEnhancedMetaFile::LINUX_IMG_WIDTH() { return LINUX_IMG_WIDTH_value; }
void CEnhancedMetaFile::LINUX_IMG_HEIGHT(int value)
{
	LINUX_IMG_HEIGHT_value = value;
}
int CEnhancedMetaFile::LINUX_IMG_HEIGHT() { return LINUX_IMG_HEIGHT_value; }
/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CEnhancedMetaFile::CEnhancedMetaFile(
	const std::string& targetFileName, int scaleFactor)
	: m_scale(scaleFactor), m_targetFile(targetFileName)
{
#ifdef _WIN32
	m_hdc =
		CreateEnhMetaFileA(nullptr, targetFileName.c_str(), nullptr, nullptr);
	if (!m_hdc.get()) THROW_EXCEPTION("Can't create EMF file!!!");
#else
	m_hdc = (void*)new CImage(LINUX_IMG_WIDTH_value, LINUX_IMG_HEIGHT_value);
	((CImage*)m_hdc.get())
		->filledRectangle(
			0, 0, LINUX_IMG_WIDTH_value - 1, LINUX_IMG_HEIGHT_value - 1,
			TColor(0, 0, 0));
#endif
}

/*---------------------------------------------------------------
						Destructor
---------------------------------------------------------------*/
CEnhancedMetaFile::~CEnhancedMetaFile()
{
#ifdef _WIN32
	// Free objects:
	if (m_hFont.get())
	{
		DeleteObject(m_hFont.get());
		m_hFont = nullptr;
	}

	// Finish EMF:
	DeleteEnhMetaFile(CloseEnhMetaFile((HDC)m_hdc.get()));
#else
	((CImage*)m_hdc.get())->saveToFile(m_targetFile + ".png");

	// Free objects:
	delete ((CImage*)m_hdc.get());
#endif
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void CEnhancedMetaFile::drawImage(int x, int y, const mrpt::img::CImage& img)
{
#ifdef _WIN32
	try
	{
		LPBITMAPINFO pBmpInfo = (LPBITMAPINFO) new unsigned char
			[sizeof(BITMAPINFOHEADER) + (256 * sizeof(RGBQUAD))];
		//		LPBITMAPINFO		pBmpInfo = (LPBITMAPINFO) new unsigned
		// char[sizeof(BITMAPINFOHEADER) ];

		unsigned int imgWidth = (unsigned int)img.getWidth();
		unsigned int imgHeight = (unsigned int)img.getHeight();

		pBmpInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		pBmpInfo->bmiHeader.biWidth = imgWidth;
		pBmpInfo->bmiHeader.biHeight = imgHeight;
		pBmpInfo->bmiHeader.biPlanes = 1;
		//		pBmpInfo->bmiHeader.biBitCount		= 24;
		pBmpInfo->bmiHeader.biBitCount = 8;
		pBmpInfo->bmiHeader.biCompression = BI_RGB;
		pBmpInfo->bmiHeader.biSizeImage = 0;
		pBmpInfo->bmiHeader.biXPelsPerMeter =
			pBmpInfo->bmiHeader.biYPelsPerMeter = 0;
		pBmpInfo->bmiHeader.biClrUsed = 0;
		pBmpInfo->bmiHeader.biClrImportant = 0;

		// Palette
		for (unsigned char i = 0; i < 255; i++)
		{
			pBmpInfo->bmiColors[i].rgbRed = i;
			pBmpInfo->bmiColors[i].rgbGreen = i;
			pBmpInfo->bmiColors[i].rgbBlue = i;
			pBmpInfo->bmiColors[i].rgbReserved = 0;
		}

		//		unsigned int	lineBytes = 3*bmp.Width;
		unsigned int lineBytes = imgWidth;
		if (lineBytes % 2) lineBytes++;
		if (lineBytes % 4) lineBytes += 2;

		BYTE* ptrBits = new BYTE[lineBytes * imgHeight];

		for (unsigned int py = 0; py < imgHeight; py++)
			for (unsigned int px = 0; px < imgWidth; px++)
				ptrBits[(py * lineBytes + px) + 0] = *img(px, py);

		HBITMAP hBitmap = CreateDIBitmap(
			(HDC)m_hdc.get(), &pBmpInfo->bmiHeader, CBM_INIT, ptrBits, pBmpInfo,
			DIB_RGB_COLORS);

		ASSERT_(hBitmap != nullptr);

		BITMAP bm;
		GetObject(hBitmap, sizeof(bm), &bm);

		HDC hdcMem = CreateCompatibleDC((HDC)m_hdc.get());
		HBITMAP hbmT = (HBITMAP)SelectObject(hdcMem, hBitmap);

		BitBlt(
			(HDC)m_hdc.get(), x, y, (int)(m_scale * imgWidth),
			(int)(m_scale * imgHeight), hdcMem, 0, 0, SRCCOPY);

		SelectObject(hdcMem, hbmT);
		DeleteDC(hdcMem);

		// Free mem:
		// ---------------------------------------
		DeleteObject(hBitmap);
		delete[] pBmpInfo;
		delete[] ptrBits;
	}
	catch (...)
	{
		THROW_EXCEPTION("Unexpected runtime error!!");
	}
#else
	((CImage*)m_hdc.get())->drawImage(x, y, img);
#endif
}

/*---------------------------------------------------------------
						drawBitmap
---------------------------------------------------------------*/
void CEnhancedMetaFile::line(
	int x0, int y0, int x1, int y1, const mrpt::img::TColor color,
	unsigned int width, TPenStyle penStyle)
{
#ifdef _WIN32
	x0 *= m_scale;
	y0 *= m_scale;
	x1 *= m_scale;
	y1 *= m_scale;

	HPEN hPen = CreatePen(penStyle, width, (unsigned int)color);

	HPEN hOldPen = (HPEN)SelectObject((HDC)m_hdc.get(), hPen);

	MoveToEx((HDC)m_hdc.get(), x0, y0, nullptr);
	LineTo((HDC)m_hdc.get(), x1, y1);

	SelectObject((HDC)m_hdc.get(), hOldPen);
	DeleteObject(hPen);
#else
	((CImage*)m_hdc.get())->line(x0, y0, x1, y1, color, width, penStyle);
#endif
}

/*---------------------------------------------------------------
						drawBitmap
---------------------------------------------------------------*/
void CEnhancedMetaFile::textOut(
	int x0, int y0, const std::string& str, const mrpt::img::TColor color)
{
#ifdef _WIN32
	x0 *= m_scale;
	y0 *= m_scale;

	::SetBkMode((HDC)m_hdc.get(), TRANSPARENT);
	::SetTextColor((HDC)m_hdc.get(), (unsigned int)color);

	::TextOutA((HDC)m_hdc.get(), x0, y0, str.c_str(), (int)str.size());
#else
	((CImage*)m_hdc.get())->textOut(x0, y0, str, color);
#endif
}

/*---------------------------------------------------------------
					selectVectorTextFont
---------------------------------------------------------------*/
void CEnhancedMetaFile::selectVectorTextFont(
	const std::string& fontName, int fontSize, bool bold, bool italic)
{
#ifdef _WIN32
	HFONT hFont, oldFont;
	LOGFONTA lpf;

	lpf.lfHeight = fontSize;
	lpf.lfWidth = 0;
	lpf.lfEscapement = 0;
	lpf.lfOrientation = 0;
	lpf.lfWeight = bold ? 700 : 400;
	lpf.lfItalic = italic ? 1 : 0;
	lpf.lfUnderline = 0;
	lpf.lfStrikeOut = 0;
	lpf.lfCharSet = DEFAULT_CHARSET;
	lpf.lfOutPrecision = OUT_DEFAULT_PRECIS;
	lpf.lfClipPrecision = CLIP_DEFAULT_PRECIS;
	lpf.lfQuality = DEFAULT_QUALITY;
	lpf.lfPitchAndFamily = DEFAULT_PITCH;
	os::strcpy(lpf.lfFaceName, LF_FACESIZE, fontName.c_str());

	hFont = ::CreateFontIndirectA(&lpf);

	oldFont = (HFONT)::SelectObject((HDC)m_hdc.get(), hFont);

	if (oldFont) ::DeleteObject(oldFont);
#else
	MRPT_UNUSED_PARAM(fontSize);
	MRPT_UNUSED_PARAM(bold);
	MRPT_UNUSED_PARAM(italic);
	MRPT_TRY_START;

	((CImage*)m_hdc.get())->selectTextFont(fontName);

	MRPT_TRY_END;
#endif
}

/*---------------------------------------------------------------
					setPixel
---------------------------------------------------------------*/
void CEnhancedMetaFile::setPixel(int x, int y, size_t color)
{
#ifdef _WIN32
	::SetPixel((HDC)m_hdc.get(), x * m_scale, y * m_scale, color);
#else
	((CImage*)m_hdc.get())->setPixel(x, y, color);
#endif
}

/*---------------------------------------------------------------
						rectangle
---------------------------------------------------------------*/
void CEnhancedMetaFile::rectangle(
	int x0, int y0, int x1, int y1, TColor color, unsigned int width)
{
	line(x0, y0, x1, y0, color, width);
	line(x1, y0, x1, y1, color, width);
	line(x1, y1, x0, y1, color, width);
	line(x0, y1, x0, y0, color, width);
}
