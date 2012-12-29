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
