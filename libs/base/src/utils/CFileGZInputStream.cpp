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

	if (m_f) gzclose(m_f);

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
		gzclose(m_f);
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

	return gzread(m_f,Buffer,Count);
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
	return gztell(m_f);
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
	else		return 0!=gzeof(m_f);
}
