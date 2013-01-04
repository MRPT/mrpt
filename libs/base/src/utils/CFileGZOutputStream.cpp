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

#include <mrpt/base.h>  // Precompiled headers


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
