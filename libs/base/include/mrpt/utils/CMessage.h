/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef  CMessage_H
#define  CMessage_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		/** A class that contain generic messages, that can be sent and received from a "CClientTCPSocket" object.
		  *  A message consists of a "header" (or type), and a "body" (or content).
		  *  Apart from arbitrary data, specific methods are provided for easing the serialization of MRPT's "CSerializable" objects.
		  *  This class is also used for passing data to hardware interfaces (see )
		  * \sa CClientTCPSocket
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CMessage
		{
		public:
			/** An identifier of the message type.
			  */
			uint32_t						type;

			/** The contents of the message (memory is automatically handled by the std::vector object)
			  */
			std::vector<unsigned char>	content;

			/** A method for serializing a MRPT's object into the content.
			  *  Any modification to data in "content" after this will corrupt the object serialization.
			  *  Member "type" is unmodified in this method.
			  */
			void  serializeObject( CSerializable *obj );

			/** A method that parse the data in the message into an existing object.
			  *  Note that the class of the object must be known and must match the one of the serialized object.
			  * \except std::exception On corrupt data, unknown serialized objects, unknown serialized object version, non-matching classes,...
			  */
			void  deserializeIntoExistingObject( CSerializable *obj );

			/** A method that parse the data in the message into a new object of (a priori) unknown class.
			  *  The pointer will contain on return a copy of the reconstructed object. Deleting this object when
			  *   no longer required is the responsability of the user. Note that previous contents of the pointer
			  *   will be ignored (it should be NULL).
			  * \except std::exception On corrupt data, unknown serialized objects, unknown serialized object version,...
			  */
			void  deserializeIntoNewObject( CSerializablePtr &obj );

			/** Sets the contents of the message from a string
			  * \sa getContentAsString
			  */
			void  setContentFromString( const std::string &str );

			/** Gets the contents of the message as a string
			  * \sa setContentFromString
			  */
			void  getContentAsString( std::string &str );

			/** Sets the contents of the message from a "void*" (the pointer itself becomes the message) - This is intended for inter-thread comms only.
			  * \sa getContentAsPointer
			  */
			void  setContentFromPointer( void * ptr );

			/** Gets the contents of the message as a "void*" (the pointer itself is the message) - This is intended for inter-thread comms only.
			  * \sa setContentFromPointer
			  */
			void * getContentAsPointer() const;

			/** Sets the contents of the message from an arbitary structure - This is intended for inter-thread comms only, the message will be not cross-platform.
			  * \sa getContentAsStruct
			  */
			template <class T>
			void  setContentFromStruct( const T &data )
			{
				content.resize( sizeof(data) );
				T * ptr = reinterpret_cast< T* >( &content[0] );
				*ptr = data;
			}

			/** Gets the contents of the message as an arbitary structure - This is intended for inter-thread comms only, the message will be not cross-platform.
			  * \sa setContentFromStruct
			  */
			template <class T>
			void getContentAsStruct( T &data ) const
			{
				MRPT_START
				ASSERT_(content.size() == sizeof(data) );
				data = * reinterpret_cast< T* >( &content[0] );
				MRPT_END
			}


		}; // End of class

	} // End of namespace
} // End of namespace

#endif
