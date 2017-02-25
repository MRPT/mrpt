/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#ifdef _MSC_VER
#	define _SCL_SECURE_NO_WARNINGS
#endif

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>

#if MRPT_HAS_GZ_STREAMS

#include <zlib.h>

#define THE_GZFILE   reinterpret_cast<gzFile>(m_f)

using namespace mrpt::utils;
using namespace std;


/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileGZOutputStream::CFileGZOutputStream( const string	&fileName ) :
	m_f(NULL)
{
	MRPT_START
	if (!open(fileName))
		THROW_EXCEPTION_CUSTOM_MSG1( "Error trying to open file: '%s'",fileName.c_str() );
	MRPT_END
}

/*---------------------------------------------------------------
				Constructor
 ---------------------------------------------------------------*/
CFileGZOutputStream::CFileGZOutputStream( ) :
	m_f(NULL)
{
}

/*---------------------------------------------------------------
							open
 ---------------------------------------------------------------*/
bool CFileGZOutputStream::open( const string	&fileName, int compress_level )
{
	MRPT_START

	if (m_f) gzclose(THE_GZFILE);

	// Open gz stream:
	m_f = gzopen(fileName.c_str(),format("wb%i",compress_level).c_str() );
	return m_f != NULL;

	MRPT_END
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CFileGZOutputStream::~CFileGZOutputStream()
{
	close();
}

/*---------------------------------------------------------------
							close
 ---------------------------------------------------------------*/
void CFileGZOutputStream::close()
{
	if (m_f)
	{
		gzclose(THE_GZFILE);
		m_f = NULL;
	}
}

/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t  CFileGZOutputStream::Read(void *Buffer, size_t Count)
{
	MRPT_UNUSED_PARAM(Buffer); MRPT_UNUSED_PARAM(Count);
	THROW_EXCEPTION("Trying to read from an output file stream.");
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t  CFileGZOutputStream::Write(const void *Buffer, size_t Count)
{
	if (!m_f) { THROW_EXCEPTION("File is not open."); }
	return gzwrite(THE_GZFILE,const_cast<void*>(Buffer),Count);
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CFileGZOutputStream::getPosition()
{
	if (!m_f) { THROW_EXCEPTION("File is not open."); }
	return gztell(THE_GZFILE);
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool  CFileGZOutputStream::fileOpenCorrectly()
{
	return m_f!=NULL;
}

#endif  // MRPT_HAS_GZ_STREAMS
