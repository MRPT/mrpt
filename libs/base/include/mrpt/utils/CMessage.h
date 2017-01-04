/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CMessage_H
#define  CMessage_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <vector>

namespace mrpt
{
	namespace utils
	{
		class CSerializable; struct CSerializablePtr; 

		/** A class that contain generic messages, that can be sent and received from a "CClientTCPSocket" object.
		  *  A message consists of a "header" (or type), and a "body" (or content).
		  *  Apart from arbitrary data, specific methods are provided for easing the serialization of MRPT's "CSerializable" objects.
		  *  This class is also used for passing data to hardware interfaces (see mrpt::hwdrivers::CSerialPort) 
		  * \sa CClientTCPSocket
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CMessage
		{
		public:
			uint32_t             type;  //!< An identifier of the message type (only the least-sig byte is typically sent)
			std::vector<uint8_t> content; //!< The contents of the message (memory is automatically handled by the std::vector object)

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
