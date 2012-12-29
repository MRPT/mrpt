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


#ifdef _MSC_VER
#	define _SCL_SECURE_NO_WARNINGS
#endif

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>


#include <zlib.h>

using namespace mrpt::utils;
using namespace std;

#define THE_GZFILE   reinterpret_cast<gzFile>(m_f)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileGZInputStream::CFileGZInputStream( const string &fileName ) : m_f(NULL)
{
	MRPT_START
	open(fileName);
	MRPT_END
}
/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileGZInputStream::CFileGZInputStream( ) : m_f(NULL)
{
}

/*---------------------------------------------------------------
							open
 ---------------------------------------------------------------*/
bool CFileGZInputStream::open(const std::string &fileName )
{
	MRPT_START

	if (m_f) gzclose(THE_GZFILE);

	// Get compressed file size:
	m_file_size = mrpt::system::getFileSize(fileName);
	if (m_file_size==uint64_t(-1))
		THROW_EXCEPTION_CUSTOM_MSG1("Couldn't access the file '%s'",fileName.c_str() );

	// Open gz stream:
	m_f = gzopen(fileName.c_str(),"rb");
	return m_f != NULL;

	MRPT_END
}

/*---------------------------------------------------------------
							close
 ---------------------------------------------------------------*/
void CFileGZInputStream::close()
{
	if (m_f)
	{
		gzclose(THE_GZFILE);
		m_f = NULL;
	}
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CFileGZInputStream::~CFileGZInputStream()
{
	close();
}

/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t  CFileGZInputStream::Read(void *Buffer, size_t Count)
{
	if (!m_f) { THROW_EXCEPTION("File is not open."); }

	return gzread(THE_GZFILE,Buffer,Count);
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t  CFileGZInputStream::Write(const void *Buffer, size_t Count)
{
	THROW_EXCEPTION("Trying to write to an input file stream.");
}

/*---------------------------------------------------------------
						getTotalBytesCount
 ---------------------------------------------------------------*/
uint64_t CFileGZInputStream::getTotalBytesCount()
{
	if (!m_f) { THROW_EXCEPTION("File is not open."); }
	return m_file_size;
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CFileGZInputStream::getPosition()
{
	if (!m_f) { THROW_EXCEPTION("File is not open."); }
	return gztell(THE_GZFILE);
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool  CFileGZInputStream::fileOpenCorrectly()
{
	return m_f!=NULL;
}

/*---------------------------------------------------------------
						checkEOF
 ---------------------------------------------------------------*/
bool CFileGZInputStream::checkEOF()
{
	if (!m_f)	return true;
	else		return 0!=gzeof(THE_GZFILE);
}
