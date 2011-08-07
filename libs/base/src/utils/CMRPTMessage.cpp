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



#include <mrpt/utils/CMessage.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CMemoryStream.h>

using namespace mrpt::utils;

/*---------------------------------------------------------------
					serializeObject
 ---------------------------------------------------------------*/
void  CMessage::serializeObject( CSerializable *obj )
{
	MRPT_START
	CMemoryStream		auxStream;

	// Dump the object in the memory stream:
	auxStream.WriteObject( obj );

	// Copy data to message:
	content.resize( auxStream.getTotalBytesCount() );
	memcpy(
		&content[0],					// Dest
		auxStream.getRawBufferData(),	// Src
		content.size() );

	MRPT_END
}

/*---------------------------------------------------------------
				deserializeIntoExistingObject
 ---------------------------------------------------------------*/
void  CMessage::deserializeIntoExistingObject( CSerializable *obj )
{
	MRPT_START
	CMemoryStream		auxStream;

	// Copy data into the stream:
	auxStream.WriteBuffer( &content[0], content.size() );
	auxStream.Seek(0);

	// Try to parse data into existing object:
	auxStream.ReadObject( obj );

	MRPT_END
}

/*---------------------------------------------------------------
				deserializeIntoNewObject
 ---------------------------------------------------------------*/
void  CMessage::deserializeIntoNewObject( CSerializablePtr &obj )
{
	MRPT_START
	CMemoryStream		auxStream;

	// Copy data into the stream:
	if (!content.empty())
	{
		auxStream.WriteBuffer( &content[0], content.size() );
		auxStream.Seek(0);

		// Try to parse data into a new object:
		obj = auxStream.ReadObject( );
	}
	else obj.clear_unique();


	MRPT_END
}

/*---------------------------------------------------------------
				setContentFromString
 ---------------------------------------------------------------*/
void  CMessage::setContentFromString( const std::string &str )
{
	content.resize( str.size() );
	if (content.size()>0)
		memcpy( &content[0], str.c_str(), str.size() );
}

/*---------------------------------------------------------------
				getContentAsString
 ---------------------------------------------------------------*/
void  CMessage::getContentAsString( std::string &str )
{
	str.resize( content.size() );
	if (content.size()>0)
		memcpy( &str[0], &content[0], str.size() );
}

/*---------------------------------------------------------------
				setContentFromPointer
 ---------------------------------------------------------------*/
void  CMessage::setContentFromPointer( void * ptr )
{
	content.resize( sizeof(void*) );
	void ** ptrPtr = reinterpret_cast<void**>( &content[0] );
	*ptrPtr = ptr;
}

/*---------------------------------------------------------------
				getContentAsPointer
 ---------------------------------------------------------------*/
void *CMessage::getContentAsPointer() const
{
	MRPT_START
	ASSERT_( content.size() == sizeof(void*) );

	return * reinterpret_cast<void**>(  const_cast<unsigned char*>( &content[0] ) ) ;

	MRPT_END
}
