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

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>

#if MRPT_HAS_GZ_STREAMS

#include <zlib.h>


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

	if (m_f) gzclose(m_f);

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
		gzclose(m_f);
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
	return gzwrite(m_f,(void*)Buffer,Count);
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CFileGZOutputStream::getPosition()
{
	if (!m_f) { THROW_EXCEPTION("File is not open."); }
	return gztell(m_f);
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool  CFileGZOutputStream::fileOpenCorrectly()
{
	return m_f!=NULL;
}

#endif  // MRPT_HAS_GZ_STREAMS
