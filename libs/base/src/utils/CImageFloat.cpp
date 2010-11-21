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

#include <mrpt/base.h>  // Precompiled headers

MRPT_TODO("JL says: Delete this class")

#include <mrpt/utils/CImageFloat.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/utils/CImage.h>


using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CImageFloat, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CImageFloat::CImageFloat( size_t	width, size_t	height) :
	m_img(NULL),
	m_width(0),
	m_height(0)
{
	resize( width, height );
}

/*---------------------------------------------------------------
						Copy constructor
 ---------------------------------------------------------------*/
CImageFloat::CImageFloat( const CImageFloat &o ) :
	m_img(NULL),
	m_width(0),
	m_height(0)
{
	if (this==&o) return;

	MRPT_START;

	resize(o.m_width,o.m_height);
	memcpy(m_img,o.m_img,sizeof(float)*m_height*m_width);

	MRPT_END;
}

/*---------------------------------------------------------------
			Copy constructor from a matrix
 ---------------------------------------------------------------*/
CImageFloat::CImageFloat( const CMatrixFloat &o ) :
	m_img(NULL),
	m_width(0),
	m_height(0)
{
	MRPT_START;
	resize(o.getColCount(),o.getRowCount());
	*this = o;
	MRPT_END;
}

/*---------------------------------------------------------------
			Copy constructor from a matrix
 ---------------------------------------------------------------*/
CImageFloat::CImageFloat( const CMatrixDouble &o ) :
	m_img(NULL),
	m_width(0),
	m_height(0)
{
	MRPT_START;
	resize(o.getColCount(),o.getRowCount());
	*this = o;
	MRPT_END;
}


/*---------------------------------------------------------------
						Copy constructor:
 ---------------------------------------------------------------*/
CImageFloat::CImageFloat( const CImage &o)
{
	MRPT_START;

	m_img = NULL;
	m_width = m_height = 0;

	(*this) = o;

	MRPT_END;
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
void  CImageFloat::operator = (const CImageFloat& o)
{
	if (this==&o) return;

#ifdef _DEBUG
	MRPT_START;
#endif

		resize(o.m_width,o.m_height);
		memcpy(m_img,o.m_img,sizeof(float)*m_height*m_width);

#ifdef _DEBUG
	MRPT_END;
#endif
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
void  CImageFloat::operator = (const CImage& o)
{
	size_t	nCols = o.getWidth();
	size_t	nRows = o.getHeight();

	resize(nCols,nRows);

	if (o.isOriginTopLeft())
	{
		for (size_t row=0;row<nRows;row++)
			for (size_t col=0;col<nCols;col++)
				m_img[col + row*nCols] =  o.getAsFloat(col,row);
	}
	else
	{
		for (size_t row=0;row<nRows;row++)
			for (size_t col=0;col<nCols;col++)
				m_img[col + ((nRows-1)-row)*nCols] =  o.getAsFloat(col,row);
	}
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
void  CImageFloat::operator = (const CMatrixFloat& o)
{
	size_t	nCols = o.getColCount();
	size_t	nRows = o.getRowCount();

	resize(nCols,nRows);

	for (size_t r=0;r<m_height;r++)
	{
		float	*destPtr = (*this)(0,r);
		float	val;
		for (size_t c=0;c<m_width;c++)
		{
			val = o.get_unsafe(r,c);
			if (val<0 || val>1)
			{
				THROW_EXCEPTION("The matrix should contain values in the range [0,1] only!");
			}
			*destPtr++  = val;
		}
	}
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
void  CImageFloat::operator = (const CMatrixDouble& o)
{
	size_t	nCols = o.getColCount();
	size_t	nRows = o.getRowCount();

	resize(nCols,nRows);

	for (size_t r=0;r<m_height;r++)
	{
		float	*destPtr = (*this)(0,r);
		float	val;
		for (size_t c=0;c<m_width;c++)
		{
			val = o.get_unsafe(r,c);
			if (val<0 || val>1)
			{
				THROW_EXCEPTION("The matrix should contain values in the range [0,1] only!");
			}
			*destPtr++  = val;
		}
	}
}


/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CImageFloat::~CImageFloat( )
{
	MRPT_START;

	free(m_img);
	m_width = m_height = 0;

	MRPT_END;
}

/*---------------------------------------------------------------
						resize
 ---------------------------------------------------------------*/
void  CImageFloat::resize(
		size_t	width,
		size_t	height)
{
	// TODO: Resize!!!
	MRPT_START;

	m_img = (float*)realloc( m_img, sizeof(float) * height * width );

	m_height = height;
	m_width  = width;

	MRPT_END;
}

/*---------------------------------------------------------------
						setSize
 ---------------------------------------------------------------*/
void  CImageFloat::setSize(
		size_t	width,
		size_t	height)
{
	MRPT_START;

	m_img = (float*)realloc( m_img, sizeof(float) * height * width );
	memset(m_img,0,sizeof(float) * height * width );

	m_height = height;
	m_width  = width;

	MRPT_END;
}


/*---------------------------------------------------------------
						operator()
 ---------------------------------------------------------------*/
float*  CImageFloat::operator() (
			size_t	col,
			size_t	row )  const
{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_START;

	if (row>=m_height || col >=m_width)
		THROW_EXCEPTION("Pixel coordinates out of bounds");
#endif

	return m_img + row*m_width + col;

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_END;
#endif
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CImageFloat::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << (uint32_t)m_width << (uint32_t)m_height;
		out.WriteBuffer(m_img, sizeof(float)*m_height*m_width);
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CImageFloat::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	width, height;

			in >> width >> height;

			resize(width,height);
			in.ReadBuffer( m_img, sizeof(float)*m_height*m_width  );

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
						getWidth
 ---------------------------------------------------------------*/
size_t  CImageFloat::getWidth() const
{
	return m_width;
}

/*---------------------------------------------------------------
						getHeight
 ---------------------------------------------------------------*/
size_t  CImageFloat::getHeight() const
{
	return m_height;
}

/*---------------------------------------------------------------
						saveToTextFile
 ---------------------------------------------------------------*/
void  CImageFloat::saveToTextFile(const std::string &fileName) const
{
	FILE	*f=os::fopen(fileName.c_str(),"wt");
	if (!f) return;

	size_t	nCols = m_width;
	size_t	nRows = m_height;

	for (size_t row=0;row<nRows;row++)
	{
		for (size_t col=0;col<nCols;col++)
		{
			os::fprintf(f,"%.4f ", m_img[ row*m_width + col] );
		}
		os::fprintf(f,"\n");
	}

	os::fclose(f);
}

/*---------------------------------------------------------------
						adjustImageRange
 ---------------------------------------------------------------*/
void  CImageFloat::adjustImageRange(float Min, float Max)
{
	size_t	nCols = m_width;
	size_t	nRows = m_height;
	size_t	row, col;

	// Find current min/max:
	// ----------------------------
	float			currentMin = 1e10, currentMax=-1e10;
	for (row=0;row<nRows;row++)
	{
		for (col=0;col<nCols;col++)
		{
			currentMin = min( currentMin, m_img[ row*m_width + col] );
			currentMax = max( currentMax, m_img[ row*m_width + col] );
		}
	}


	// Adjust: pixel_new = a + b · pixel_old
	// --------------------------------------
	float	a,b;
	if (currentMin==currentMax)
			b = 0;
	else	b = (Max-Min) / (currentMax-currentMin);
	a = Min - b * currentMin;

	for (row=0;row<nRows;row++)
		for (col=0;col<nCols;col++)
			m_img[ row*m_width + col] = a + b * m_img[ row*m_width + col];

}

/*---------------------------------------------------------------
						getAsMatrix
 ---------------------------------------------------------------*/
void  CImageFloat::getAsMatrix( CMatrixFloat &outMatrix )  const
{
	outMatrix.setSize( m_height, m_width );

	float	*ptr = m_img;

	for (size_t y=0;y<m_height;y++)
		for (size_t x=0;x<m_width;x++)
			outMatrix.set_unsafe(y,x, *ptr++ );
}

/*---------------------------------------------------------------
						setPixel
 ---------------------------------------------------------------*/
void  CImageFloat::setPixel(int x, int y, size_t color)
{
	if (x<0 || x>=(int)m_width || y<0 || y>=(int)m_height)	return;
	m_img[ y*m_width + x ] = ((unsigned char)color) / 255.0f;
}

/*---------------------------------------------------------------
						saveToFile
 ---------------------------------------------------------------*/
bool  CImageFloat::saveToFile(const std::string &fileName,bool verticalFlip) const
{
	MRPT_START
	CImage	auxImg(*this);
	auxImg.setOriginTopLeft( !verticalFlip );
	return auxImg.saveToFile(fileName);
	MRPT_END
}

/*---------------------------------------------------------------
						loadFromFile
 ---------------------------------------------------------------*/
bool  CImageFloat::loadFromFile( const std::string& fileName  )
{
	MRPT_START;

	CImage	auxImg;

	if (!auxImg.loadFromFile(fileName,0 /*force grayscale*/) )
		return false;

	if (auxImg.isColor())
		THROW_EXCEPTION("The image file must be grayscale!");

	(*this) = auxImg;

	return true;
	MRPT_END;
}

CImageFloat  CImageFloat::operator*( const CImageFloat &im2 )
{
	ASSERT_(m_width == im2.getWidth());
	ASSERT_(m_height == im2.getHeight());

	CImageFloat temp(m_width, m_height);
	for (size_t j = 0; j < m_height; j++) {
		float *ptrOut = temp(0,j);
		float *ptrIn = im2(0,j);
		for (size_t i = 0; i < m_width; i++)
			(*ptrOut++) = m_img[j*m_width + i]*(*ptrIn++);

	}

	/*tictac.Tic();
	for (size_t i = 0; i < m_width; i++)
		for (size_t j = 0; j < m_height; j++)
			(*temp(i,j)) = m_img[j*m_width + i]*(*im2(i,j));

	std::cout << "No Punteros: " << tictac.Tac()*1000 << std::endl;*/

	return temp;
}

CImageFloat  CImageFloat::operator+(const CImageFloat &im2 )
{
	ASSERT_(m_width == im2.getWidth());
	ASSERT_(m_height == im2.getHeight());

	CImageFloat temp(m_width, m_height);
	for (size_t j = 0; j < m_height; j++) {
		float *ptrOut = temp(0,j);
		float *ptrIn = im2(0,j);
		for (size_t i = 0; i < m_width; i++)
			(*ptrOut++) = m_img[j*m_width + i]+(*ptrIn++);

	}

	return temp;
}

CImageFloat  CImageFloat::operator-(const CImageFloat &im2 )
{
	ASSERT_(m_width == im2.getWidth());
	ASSERT_(m_height == im2.getHeight());

	CImageFloat temp(m_width, m_height);
	for (size_t j = 0; j < m_height; j++) {
		float *ptrOut = temp(0,j);
		float *ptrIn = im2(0,j);
		for (size_t i = 0; i < m_width; i++)
			(*ptrOut++) = m_img[j*m_width + i]-(*ptrIn++);

	}

	return temp;
}


CImageFloat  CImageFloat::operator^( int exp )
{
	CImageFloat temp(m_width, m_height);
//	CTicTac	tictac;
//	tictac.Tic();
	for (size_t j = 0; j < m_height; j++) {
		float *ptrOut = temp(0,j);
		for (size_t i = 0; i < m_width; i++)
			(*ptrOut++) = pow(m_img[j*m_width + i],exp);

	}
//	std::cout << "Punteros: " << tictac.Tac()*1000 << std::endl;

	return temp;
}

void  CImageFloat::scaleHalf()
{
	size_t	x,y;
	size_t	newHeight = m_height >> 1;
	size_t	newWidth  = m_width  >> 1;

	for (y=0;y<newHeight;y++)
	{
		float *ptrSrc = &m_img[ (2*y)*m_width ];
		float *ptrDst = &m_img[ y*newWidth ];

		for (x=0;x<newWidth;x++)
		{
			*ptrDst++ = *ptrSrc++;
			ptrSrc++;
		}
	}

	m_width  = newWidth;
	m_height = newHeight;
	m_img    = (float*)realloc( m_img, sizeof(float) * m_height * m_width );
}
