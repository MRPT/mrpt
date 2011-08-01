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

#include <mrpt/opengl.h>  // Precompiled header


#include <mrpt/opengl/CTexturedObject.h>
#include <mrpt/utils/CTimeLogger.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CTexturedObject, CRenderizableDisplayList, mrpt::opengl )

// Whether to profile memory allocations:
//#define TEXTUREOBJ_PROFILE_MEM_ALLOC

// Whether to use a memory pool for the texture buffer:
#define TEXTUREOBJ_USE_MEMPOOL

// Data types for memory pooling CTexturedObject:
#ifdef TEXTUREOBJ_USE_MEMPOOL

#	include <mrpt/system/CGenericMemoryPool.h>

	struct CTexturedObject_MemPoolParams
	{
		size_t len; //!< size of the vector<unsigned char>

		inline bool isSuitable(const CTexturedObject_MemPoolParams &req) const {
			return len==req.len;
		}
	};
	struct CTexturedObject_MemPoolData
	{
		vector<unsigned char> data;
	};

	typedef CGenericMemoryPool<CTexturedObject_MemPoolParams,CTexturedObject_MemPoolData> TMyMemPool;
#endif


/*---------------------------------------------------------------
							CTexturedObject
  ---------------------------------------------------------------*/
CTexturedObject::CTexturedObject() :
	m_glTextureName(0),
	m_texture_is_loaded(false),
	m_enableTransparency(false)
{
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void  CTexturedObject::assignImage(
	const CImage& img,
	const CImage& imgAlpha )
{
	MRPT_START

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage = img;
	m_textureImageAlpha = imgAlpha;

	m_enableTransparency = true;

	MRPT_END
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void  CTexturedObject::assignImage(
	const CImage& img )
{
	MRPT_START;

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage = img;

	m_enableTransparency = false;

	MRPT_END;
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void  CTexturedObject::assignImage_fast(
	CImage& img,
	CImage& imgAlpha )
{
	MRPT_START;

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage.copyFastFrom(img);
	m_textureImageAlpha.copyFastFrom(imgAlpha);

	m_enableTransparency = true;

	MRPT_END;
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void  CTexturedObject::assignImage_fast(
	CImage& img )
{
	MRPT_START;

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage.copyFastFrom(img);

	m_enableTransparency = false;

	MRPT_END;
}


// Auxiliary function for loadTextureInOpenGL(): reserve memory and return 16byte aligned starting point within it:
unsigned char *reserveDataBuffer(const size_t len, vector<unsigned char> &data)
{
#ifdef TEXTUREOBJ_USE_MEMPOOL
	TMyMemPool &pool = TMyMemPool::getInstance();

	CTexturedObject_MemPoolParams mem_params;
	mem_params.len = len;

	CTexturedObject_MemPoolData *mem_block = pool.request_memory(mem_params);
	if (mem_block)
	{
		// Recover the memory block via a swap:
		data.swap(mem_block->data);
		delete mem_block;
	}

#endif
	data.resize(len);
	return ((unsigned char*)(((POINTER_TYPE)&data[0]) & (~((POINTER_TYPE)0x0F)) )) + 0x10;
}


/*---------------------------------------------------------------
							loadTextureInOpenGL
  ---------------------------------------------------------------*/
void  CTexturedObject::loadTextureInOpenGL() const
{
#if MRPT_HAS_OPENGL_GLUT
	unsigned char	*dataAligned=NULL;
	vector<unsigned char>		data;

#	ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
	static mrpt::utils::CTimeLogger tim;
#	endif


	try
	{
		if (m_texture_is_loaded)
		{
			glBindTexture( GL_TEXTURE_2D, m_glTextureName );
			checkOpenGLError();
			return;
		}


		// Reserve the new one --------------------------

        // allocate texture names:
		m_glTextureName = getNewTextureNumber();

		// select our current texture
		glBindTexture( GL_TEXTURE_2D, m_glTextureName );
		checkOpenGLError();

		// when texture area is small, bilinear filter the closest mipmap
		// Other options: //  GL_LINEAR ); // _MIPMAP_NEAREST  ); // GL_LINEAR_MIPMAP_NEAREST );  // GL_LINEAR
		//  See also: http://www.opengl.org/discussion_boards/ubbthreads.php?ubb=showflat&Number=133116&page=1
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		checkOpenGLError();

		// when texture area is large, bilinear filter the first mipmap
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); //GL_LINEAR );
	    checkOpenGLError();

		// if wrap is true, the texture wraps over at the edges (repeat)
		//       ... false, the texture ends at the edges (clamp)
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		checkOpenGLError();

		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_REPEAT );
		checkOpenGLError();

		// Assure that the images do not overpass the maximum dimensions allowed by OpenGL:
		// ------------------------------------------------------------------------------------
		GLint	texSize;
		glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texSize);
		while ( m_textureImage.getHeight()>(unsigned int)texSize || m_textureImage.getWidth()>(unsigned int)texSize )
		{
			m_textureImage		= m_textureImage.scaleHalf();
			m_textureImageAlpha = m_textureImageAlpha.scaleHalf();
		}

		int		width = m_textureImage.getWidth();
		int		height = m_textureImage.getHeight();

		r_width = round2up( width );
		r_height = round2up( height );
		m_fill_x_left = (r_width - width) >> 1; // div by 2;
		m_fill_y_top = (r_height - height) >> 1; // div by 2;
		m_fill_x_right = (r_width - width) - m_fill_x_left;
		m_fill_y_bottom = (r_height - height) - m_fill_y_top;


        if (m_enableTransparency)
		{
			ASSERT_( !m_textureImageAlpha.isColor());
			ASSERT_(m_textureImageAlpha.getWidth() == m_textureImage.getWidth());
			ASSERT_(m_textureImageAlpha.getHeight() == m_textureImage.getHeight());
		}

		if (m_textureImage.isColor())
		{
			// Color texture:
			if (m_enableTransparency)
			{
				// Color texture WITH trans.
				// --------------------------------------
				const GLenum pixels_format = mrpt::system::os::_strcmp(m_textureImage.getChannelsOrder(),"BGR")==0 ? GL_BGRA : GL_RGBA;

#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				const std::string sSec = mrpt::format("opengl_texture_alloc %ix%i (color,trans)",r_width,r_height);
				tim.enter(sSec.c_str());
#			endif

				dataAligned = reserveDataBuffer(r_height*r_width*4 + 512, data );

#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				tim.leave(sSec.c_str());
#			endif

				for (int y=0;y<height;y++)
				{
					unsigned char 	*ptrSrcCol = m_textureImage(0,y,0);
					unsigned char 	*ptrSrcAlfa = m_textureImageAlpha(0,y);
					unsigned char 	*ptr = dataAligned + (m_fill_x_left)*4 + (m_fill_y_top+y)*r_width*4;

					for (int x=0;x<width;x++)
					{
    					*ptr++ = *ptrSrcCol++;
    					*ptr++ = *ptrSrcCol++;
    					*ptr++ = *ptrSrcCol++;
    					*ptr++ = *ptrSrcAlfa++;
					}
				}
				// build our texture mipmaps
				gluBuild2DMipmaps( GL_TEXTURE_2D, 4, r_width, r_height,pixels_format, GL_UNSIGNED_BYTE, dataAligned );
				checkOpenGLError();
			} // End of color texture WITH trans.
			else
			{
				// Color texture WITHOUT trans.
				// --------------------------------------
				const GLenum pixels_format = mrpt::system::os::_strcmp(m_textureImage.getChannelsOrder(),"BGR")==0 ? GL_BGR : GL_RGB;

#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				const std::string sSec = mrpt::format("opengl_texture_alloc %ix%i (color)",r_width,r_height);
				tim.enter(sSec.c_str());
#			endif

				dataAligned = reserveDataBuffer(r_height*r_width*3 + 512, data );

#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				tim.leave(sSec.c_str());
#			endif

				const size_t bytesPerLineSrc =   width*3;
				const size_t bytesPerLineDst = r_width*3;
				unsigned char *ptrDst = dataAligned + (m_fill_x_left)*3 + m_fill_y_top*r_width*3;
				for (int y=0;y<height;y++)
				{
					memcpy(ptrDst,m_textureImage(0,y,0),bytesPerLineSrc);
					ptrDst += bytesPerLineDst;
				}

				// Build texture:
				gluBuild2DMipmaps( GL_TEXTURE_2D, 3, r_width, r_height,pixels_format, GL_UNSIGNED_BYTE, dataAligned );
				checkOpenGLError();

			} // End of color texture WITHOUT trans.
		}
		else
		{
			// Gray-scale texture:
			if (m_enableTransparency)
			{
#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				const std::string sSec = mrpt::format("opengl_texture_alloc %ix%i (gray,transp)",r_width,r_height);
				tim.enter(sSec.c_str());
#			endif

				dataAligned = reserveDataBuffer(r_height*r_width*2 + 1024, data );

#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				tim.leave(sSec.c_str());
#			endif

				for (int y=0;y<height;y++)
				{
					unsigned char 	*ptrSrcCol = m_textureImage(0,y);
					unsigned char 	*ptrSrcAlfa = m_textureImageAlpha(0,y);
					unsigned char 	*ptr = dataAligned + (m_fill_x_left)*2 + (m_fill_y_top+y)*r_width*2;
					for (int x=0;x<width;x++)
					{
    					*ptr++ = *ptrSrcCol++;
    					*ptr++ = *ptrSrcAlfa++;
					}
				}

				// build our texture mipmaps
				gluBuild2DMipmaps( GL_TEXTURE_2D, 2, r_width, r_height, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, dataAligned );
				checkOpenGLError();
			}// End of gray-scale texture WITH trans.
			else
			{
#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				const std::string sSec = mrpt::format("opengl_texture_alloc %ix%i (gray)",r_width,r_height);
				tim.enter(sSec.c_str());
#			endif

				dataAligned = reserveDataBuffer(r_height*r_width + 1024, data );

#			ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				tim.leave(sSec.c_str());
#			endif

				for (int y=0;y<height;y++)
				{
					unsigned char 	*ptrSrcCol = m_textureImage(0,y);
					unsigned char 	*ptr = dataAligned + m_fill_x_left + (m_fill_y_top+y)*r_width;
					memcpy(ptr,ptrSrcCol, r_width);
				}

				// build our texture mipmaps
				gluBuild2DMipmaps( GL_TEXTURE_2D, 1, r_width, r_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, dataAligned );
				checkOpenGLError();
			}// End of gray-scale texture WITHOUT trans.

		}

		m_texture_is_loaded = true;

#ifdef TEXTUREOBJ_USE_MEMPOOL
		// Before freeing the buffer in "data", donate my memory to the pool:
		if (!data.empty())
		{
			TMyMemPool &pool = TMyMemPool::getInstance();

			CTexturedObject_MemPoolParams mem_params;
			mem_params.len = data.size();

			CTexturedObject_MemPoolData *mem_block = new CTexturedObject_MemPoolData();
			data.swap(mem_block->data);

			pool.dump_to_pool(mem_params, mem_block);
		}
#endif

	}
	catch(exception &e)
	{
		THROW_EXCEPTION(format("m_glTextureName=%i\n%s",m_glTextureName,e.what()));
	}
	catch(...)
	{
		THROW_EXCEPTION("Runtime error!");
	}
#endif
}

/*---------------------------------------------------------------
							~CTexturedObject
  ---------------------------------------------------------------*/
CTexturedObject::~CTexturedObject()
{
	unloadTexture();
}

/*---------------------------------------------------------------
							unloadTexture
  ---------------------------------------------------------------*/
void CTexturedObject::unloadTexture()
{
    if (m_texture_is_loaded)
    {
    	m_texture_is_loaded = false;
		releaseTextureName( m_glTextureName );
		m_glTextureName = 0;
	}
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CTexturedObject::writeToStreamTexturedObject(CStream &out) const
{
	uint8_t ver = 0;

	out << ver;
	out << m_enableTransparency;
	out << m_textureImage;
	if (m_enableTransparency) out << m_textureImageAlpha;
}

void  CTexturedObject::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	render_pre();
	if (glGetError()!= GL_NO_ERROR)
		std::cerr << "render_pre: Error" << std::endl;
	render_texturedobj();
	if (glGetError()!= GL_NO_ERROR)
		std::cerr << "render_texturedobj: Error" << std::endl;
	render_post();
	if (glGetError()!= GL_NO_ERROR)
		std::cerr << "render_post: Error" << std::endl;
#endif
}


void  CTexturedObject::render_pre() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START
	glEnable(GL_TEXTURE_2D);
	checkOpenGLError();

	if (m_enableTransparency || m_color.A!=255)
	{
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

	// This will load and/or select our texture, only if "m_texture_is_loaded" is false
	loadTextureInOpenGL();
	MRPT_END
#endif
}

void  CTexturedObject::render_post() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	if (m_enableTransparency || m_color.A!=255)
	{
		glDisable(GL_BLEND);
		checkOpenGLError();

		glBlendFunc(GL_ONE, GL_ZERO);

		glEnable(GL_DEPTH_TEST);
		checkOpenGLError();
	}

	glDisable(GL_TEXTURE_2D);
	checkOpenGLError();

	MRPT_END
#endif
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CTexturedObject::readFromStreamTexturedObject(CStream &in)
{
	uint8_t version;
	in >> version;

	CRenderizableDisplayList::notifyChange();

	switch(version)
	{
	case 0:
		{
			in >> m_enableTransparency;
			in >> m_textureImage;
			if (m_enableTransparency)
			{
				in >> m_textureImageAlpha;
				assignImage( m_textureImage, m_textureImageAlpha );
			}
			else
			{
				assignImage( m_textureImage );
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(ver)
	};
}
