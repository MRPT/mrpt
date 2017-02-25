/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CCanvas.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/utils/round.h>

#include <mrpt/compress/zip.h>
#include <map>


// Include the MRPT bitmap fonts:
#include "mrpt_font_5x7.h"
#include "mrpt_font_6x13.h"
#include "mrpt_font_6x13B.h"
#include "mrpt_font_6x13O.h"
#include "mrpt_font_9x15.h"
#include "mrpt_font_9x15B.h"
#include "mrpt_font_10x20.h"


// Japanese fonts?
#if MRPT_HAS_ASIAN_FONTS
    #include "mrpt_font_18x18ja.h"
#endif

// Each font has a block a data with this header (It's actually zip-compressed since mrpt >0.6.5)
//	const uint32_t mrpt_font_9x15B [] = {
//	9,15, /* width, height */
//	0x0000,0x00FF, /* UNICODE characters range: */


using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

//map<string,const uint32_t*>   list_registered_fonts;
map<string,vector_byte>   list_registered_fonts;   // Each vector is the target place where to uncompress each font.
bool    list_fonts_init = false;

void init_fonts_list()
{
    if (!list_fonts_init)
    {
		list_registered_fonts.clear();

	// This was used only once
#if 0
	#define SAVE_COMPRESSED( ARR )    	\
			{ \
				list_registered_fonts[#ARR].resize(sizeof(mrpt_font_##ARR));  \
				memcpy(&list_registered_fonts[#ARR][0], mrpt_font_##ARR, sizeof(mrpt_font_##ARR)); \
				cout << #ARR << " -> " <<  sizeof(mrpt_font_##ARR) << endl;  \
				CFileGZOutputStream f(string("mrpt_font_")+string(#ARR)+string(".gz")); \
				f.WriteBuffer(mrpt_font_##ARR, sizeof(mrpt_font_##ARR)); \
				/*mrpt::compress::zip::compress( list_registered_fonts[#ARR], f ); */  \
			}

    	SAVE_COMPRESSED(5x7)
//    	SAVE_COMPRESSED(6x13)
//    	SAVE_COMPRESSED(6x13B)
//    	SAVE_COMPRESSED(6x13O)
//    	SAVE_COMPRESSED(9x15)
//    	SAVE_COMPRESSED(9x15B)
//    	SAVE_COMPRESSED(10x20)

	#if MRPT_HAS_ASIAN_FONTS
//    	SAVE_COMPRESSED(18x18ja)
	#endif

#endif

#if 1  // Normal operation: Load fonts and uncompress them:

		#define LOAD_FONT(FONTNAME) \
		{ \
			vector_byte tmpBuf(sizeof(mrpt_font_gz_##FONTNAME)); \
			memcpy(&tmpBuf[0], mrpt_font_gz_##FONTNAME, sizeof(mrpt_font_gz_##FONTNAME)); \
			mrpt::compress::zip::decompress_gz_data_block(tmpBuf,list_registered_fonts[#FONTNAME]); \
		}

		LOAD_FONT(5x7)
		LOAD_FONT(6x13)
		LOAD_FONT(6x13B)
		LOAD_FONT(6x13O)
		LOAD_FONT(9x15)
		LOAD_FONT(9x15B)
		LOAD_FONT(10x20)
#if MRPT_HAS_ASIAN_FONTS
		LOAD_FONT(18x18ja)
#endif

#endif

        list_fonts_init=true;
    }
}


/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CCanvas::CCanvas() :
	m_selectedFont("9x15"),
	m_selectedFontBitmaps(NULL)
{
}

/*---------------------------------------------------------------
						line
---------------------------------------------------------------*/
void  CCanvas::line(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	const mrpt::utils::TColor	color,
	unsigned int	width,
	TPenStyle		penStyle
	)
{
	MRPT_UNUSED_PARAM(width);
	MRPT_UNUSED_PARAM(penStyle);

/*	// JL: worthy annoying so much?
	static bool warningFirst = true;
	if (warningFirst)
	{
		warningFirst=false;
		printf("[CCanvas::line] WARNING: Using default drawing method, ignoring 'width' and 'penStyle'!!\n");
	}*/

	float	x,y;

	float	Ax = (float)( x1-x0 );
	float	Ay = (float)( y1-y0 );

	// In this cases, there is nothing to do!
	if (Ax==0 && Ay==0)	return;
	if (x0<0 && x1<0)	return;
	if (y0<0 && y1<0)	return;
	if (x0>=(int)getWidth() && x1>=(int)getWidth())	return;
	if (y0>=(int)getHeight() && y1>=(int)getHeight())	return;

	float	dist  = sqrt( square(Ax)+square(Ay) );
	int		i,N = (int)ceil(dist);

	// The N steps to perform next:
	Ax/=N;	Ay/=N;
	x = (float)x0; y = (float)y0;

	for (i=0;i<N;i++)
	{
		x+= Ax; y+= Ay;
		setPixel((int)x,(int)y,color);
	} // end for i

}

/*---------------------------------------------------------------
						rectangle
---------------------------------------------------------------*/
void  CCanvas::rectangle(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	const mrpt::utils::TColor	color,
	unsigned int	width)
{
	int		w_min = (int) -ceil(((float)width)/2);
	int		w_max = (int)  floor(((float)width)/2);
	// Draw "width" rectangles one into another:
	for (int w=w_min;w<=w_max;w++)
	{
		line( x0-w,y0-w, x1+w, y0-w,  color );
		line( x1+w,y0-w, x1+w, y1+w,  color );
		line( x1+w, y1+w, x0-w, y1+w, color );
		line( x0-w, y1+w, x0-w,y0-w,  color );
	} // end for "w"
}

/*****************************************************AJOGD***************************************************/
/*---------------------------------------------------------------
						triangle
---------------------------------------------------------------*/
void  CCanvas::triangle(
			int				x0,
			int				y0,
			int				size,
			const mrpt::utils::TColor	color,
			bool			inferior,
			unsigned int	width)
{
	int ts = round(0.866*size);
	int tc = round(0.5*size);
	if (inferior)
	{
		line(x0,y0+size,x0+ts,y0-tc,color,width);
		line(x0,y0+size,x0-ts,y0-tc,color,width);
		line(x0+ts,y0-tc,x0-ts,y0-tc,color,width);
	}
	else
	{
		line(x0,y0-size,x0+ts,y0+tc,color,width);
		line(x0,y0-size,x0-ts,y0+tc,color,width);
		line(x0+ts,y0+tc,x0-ts,y0+tc,color,width);
	}
}
/************************************************************************************************************/


/*---------------------------------------------------------------
						filledRectangle
---------------------------------------------------------------*/
void  CCanvas::filledRectangle(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	const mrpt::utils::TColor	color)
{
	int		x_min = max(x0,0);
	int		x_max = min(x1,(int)getWidth()-1);
	int		y_min = max(y0,0);
	int		y_max = min(y1,(int)getHeight()-1);

	for (int y=y_min;y<=y_max;y++)
		for (int x=x_min;x<=x_max;x++)
			setPixel(x,y,color);
}

/*---------------------------------------------------------------
					selectTextFont
---------------------------------------------------------------*/
void  CCanvas::selectTextFont( const std::string  &fontName )
{
	init_fonts_list();

	// Assure list name is in the list:
	map<string,vector_byte>::const_iterator it= list_registered_fonts.find(fontName);
	if (it==list_registered_fonts.end())
	{
		// Error
		cerr << "[CCanvas::selectTextFont] Warning: Unknown font: " << fontName << endl;
		return;
	}
	else
	{
		m_selectedFontBitmaps = reinterpret_cast<const uint32_t*>( &it->second[0] );
		m_selectedFont = fontName;
	}
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void  CCanvas::drawImage(
	int						x,
	int						y,
	const utils::CImage	&img )
{
	MRPT_START

	int		img_lx = img.getWidth();
	int		img_ly = img.getHeight();

	if (img.isColor())
	{
		for (int xx=0;xx<img_lx;xx++)
			for (int yy=0;yy<img_ly;yy++)
				setPixel(x+xx,y+yy, *((int*)img(xx,yy)) );
	}
	else
	{
		unsigned char	c;
		int				col;
		for (int xx=0;xx<img_lx;xx++)
			for (int yy=0;yy<img_ly;yy++)
			{
				c = *((unsigned char *)img(xx,yy));
				col = c | (c<<8) | (c<<16);
				setPixel(x+xx,y+yy, col );
			}
	}


	MRPT_END
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void  CCanvas::drawImage(
	int						x,
	int						y,
	const utils::CImage	&img,
	float					rotation,
	float					scale )
{
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	MRPT_UNUSED_PARAM(img); MRPT_UNUSED_PARAM(rotation);
	MRPT_UNUSED_PARAM(scale);

	MRPT_START

	THROW_EXCEPTION("Not implemented yet!! Try yourself! ;-)");

	MRPT_END
}


/*---------------------------------------------------------------
						cross
---------------------------------------------------------------*/
void  CCanvas::cross(int x0,int y0, const mrpt::utils::TColor	color, char type, unsigned int size, unsigned int width)
{
	switch(type)
	{
	case '+':
		line(x0-size,y0,x0+size,y0,color,width);
		line(x0,y0-size,x0,y0+size,color,width);
		break;
	case 'x':
		line(x0-size,y0-size,x0+size,y0+size,color,width);
		line(x0+size,y0-size,x0-size,y0+size,color,width);
		break;
	case ':':
		line(x0-size,y0,x0-2,y0,color,width);
		line(x0+2,y0,x0+size,y0,color,width);
		line(x0,y0-size,x0,y0-2,color,width);
		line(x0,y0+2,x0,y0+size,color,width);
		break;
	default:
		THROW_EXCEPTION("Unexpected 'type' of cross to be drawn")
	}
}

/*---------------------------------------------------------------
						drawCircle
---------------------------------------------------------------*/
void  CCanvas::drawCircle(
	int		x,
	int		y,
	int		radius,
	const mrpt::utils::TColor	&color,
	unsigned int	width
	)
{
	if (radius<0) radius=-radius;

	int nSegments;

	if (radius==0)
	{
		nSegments=2;
	}
	else
	{
		nSegments = int(M_2PI * radius);
	}

	int								x1=0,y1=0,x2=0,y2=0;
	double							ang, Aa = M_2PI/(nSegments-1);
	int								i;

	for (i=0,ang=0;i<nSegments;i++,ang+=Aa)
	{
		x2 = round( x + radius * cos(ang) );
		y2 = round( y + radius * sin(ang) );

		if (i>0)
			line( x1, y1,x2, y2,color,width );

		x1 = x2;
		y1 = y2;
	} // end for points on ellipse
}


/*---------------------------------------------------------------
						textOut
---------------------------------------------------------------*/
void  CCanvas::textOut(
	int					x0,
	int					y0,
	const std::string	&str,
	const mrpt::utils::TColor	color
	)
{
	MRPT_START

	if (!m_selectedFontBitmaps) // First call: load fonts
		this->selectTextFont("9x15");

	// Am I an image?
	bool y_axis_reversed = false;
	CImage* im_image = dynamic_cast<CImage*>(this);
	if (im_image)
		y_axis_reversed = !im_image->isOriginTopLeft();

	// Decode UNICODE string:
	vector_word     uniStr;
	mrpt::system::decodeUTF8( str, uniStr );


	int px= x0;
	int py= y0;

	// Char size:
	uint32_t char_w = m_selectedFontBitmaps[0];
	uint32_t char_h = m_selectedFontBitmaps[1];


	for (size_t i=0; i<uniStr.size() ; i++)
	{
		const uint16_t  &unichar = uniStr[i];

		// look for the character in the table:
		const uint32_t *table_ptr = m_selectedFontBitmaps+2;
		uint32_t charset_ini = table_ptr[0];
		uint32_t charset_end = table_ptr[1];

		while (charset_end)
		{
			// Is in this range?
			if ( unichar<=charset_end && unichar>=charset_ini )
			{
				// Draw this character:
				unsigned pyy = y_axis_reversed ?  (py+char_h-1) : py;
				unsigned pxx;

				const uint32_t *char_bitmap = table_ptr+ 2 + char_h*(unichar-charset_ini);

				for (unsigned y=0;y<char_h;y++,pyy+= y_axis_reversed ? -1:1 )
				{
					pxx = px;
					const uint32_t &row = *char_bitmap++;
					for (unsigned x=0;x<char_w;x++,pxx++)
						if (row & (1 << x))
							setPixel(pxx,pyy,color);
				}

				// Advance the raster cursor:
				px+=char_w;

				// Next char!
				break;
			}
			else
			{
				// No: Move to the next block and keep searching:
				uint32_t n_chars = charset_end-charset_ini+1;
				table_ptr+= 2 /* Header */  + n_chars * char_h;

				// get new block header:
				charset_ini = table_ptr[0];
				charset_end = table_ptr[1];
			}
		}
		// Char not in the font!
	}

	MRPT_END
}



