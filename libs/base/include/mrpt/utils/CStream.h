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
#ifndef  CSTREAM_H
#define  CSTREAM_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CUncopiable.h>
#include <mrpt/utils/CObject.h>
#include <mrpt/utils/exceptions.h>

namespace mrpt
{
	namespace utils
	{
		class BASE_IMPEXP CSerializable;
		struct BASE_IMPEXP  CSerializablePtr;
		class BASE_IMPEXP CMessage;

		/** This base class is used to provide a unified interface to
		 *    files,memory buffers,..Please see the derived classes. This class is
		 *    largely inspired by Borland VCL "TStream" class. <br><br>
		 *  Apart of the "VCL like" methods, operators ">>" and "<<" have been
		 *    defined so that simple types (int,bool,char,float,char *,std::string,...)
		 *    can be directly written and read to and from any CStream easily.
		 *  Please, it is recomendable to read CSerializable documentation also.
		 *
		 * \ingroup mrpt_base_grp
		 * \sa CFileStream, CMemoryStream,CSerializable
		 */
		class BASE_IMPEXP CStream
		{
		public:
			/** Used in CStream::Seek
			  */
			enum TSeekOrigin
			{
				sFromBeginning = 0,
				sFromCurrent = 1,
				sFromEnd = 2
			};

		protected:
			/** Introduces a pure virtual method responsible for reading from the stream.
			 */
			virtual size_t  Read(void *Buffer, size_t Count) = 0;

			/** Introduces a pure virtual method responsible for writing to the stream.
			 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
			 */
			virtual size_t  Write(const void *Buffer, size_t Count) = 0;
		public:
			/* Constructor
			 */
			CStream() { }

			/* Destructor
			 */
			virtual ~CStream();

			/** Reads a block of bytes from the stream into Buffer
			 *	\exception std::exception On any error, or if ZERO bytes are read.
			 *  \return The amound of bytes actually read.
			 * \note This method is endianness-dependent. 
			 * \sa ReadBufferImmediate ; Important, see: ReadBufferFixEndianness, 
			 */
			size_t  ReadBuffer(void *Buffer, size_t Count);

			/** Reads a sequence of elemental datatypes, taking care of reordering their bytes from the MRPT stream standard (little endianness) to the format of the running architecture.
			 *  \param ElementCount The number of elements (not bytes) to read.
			 *  \param ptr A pointer to the first output element in an array (or std::vector<>, etc...).
			 *  \return The amound of *bytes* (not elements) actually read (under error situations, the last element may be invalid if the data stream abruptly ends).
			 *  Example of usage:
			 *  \code
			 *   uint32_t  N;
			 *   s >> N;
			 *   vector<float>  vec(N);
			 *   if (N) 
			 *     s.ReadBufferFixEndianness<float>(&vec[0],N);
			 *  \endcode
			 *	\exception std::exception On any error, or if ZERO bytes are read.
			 * \sa ReadBufferFixEndianness, ReadBuffer
			 */
			template <typename T>
			size_t  ReadBufferFixEndianness(T *ptr, size_t ElementCount)
			{
			#if !MRPT_IS_BIG_ENDIAN
				// little endian: no conversion needed.
				return ReadBuffer(ptr,ElementCount*sizeof(T));
			#else
				// big endian: convert.
				const size_t nread = ReadBuffer(ptr,ElementCount*sizeof(T));
				for (size_t i=0;i<ElementCount;i++) mrpt::utils::reverseBytesInPlace(ptr[i]);
				return nread;
			#endif
			}


			/** Reads a block of bytes from the stream into Buffer, and returns the amound of bytes actually read, without waiting for more extra bytes to arrive (just those already enqued in the stream).
			 *  Note that this method will fallback to ReadBuffer() in most CStream classes but in some hardware-related  classes.
			 *	\exception std::exception On any error, or if ZERO bytes are read.
			 */
			virtual size_t  ReadBufferImmediate(void *Buffer, size_t Count) { return ReadBuffer(Buffer, Count); }

			/** Writes a block of bytes to the stream from Buffer.
			 *	\exception std::exception On any error
			 *  \sa Important, see: WriteBufferFixEndianness
			 * \note This method is endianness-dependent. 
			 */
			void  WriteBuffer (const void *Buffer, size_t Count);



			/** Writes a sequence of elemental datatypes, taking care of reordering their bytes from the running architecture to MRPT stream standard (little endianness).
			 *  \param ElementCount The number of elements (not bytes) to write.
			 *  \param ptr A pointer to the first input element in an array (or std::vector<>, etc...).
			 *  Example of usage:
			 *  \code
			 *   vector<float>  vec = ...
			 *   uint32_t N = vec.size();
			 *   s << N
			 *   if (N) 
			 *     s.WriteBufferFixEndianness<float>(&vec[0],N);
			 *  \endcode
			 *  \exception std::exception On any error
			 *  \sa WriteBuffer
			 */
			template <typename T>
			void WriteBufferFixEndianness(const T *ptr, size_t ElementCount)
			{
			#if !MRPT_IS_BIG_ENDIAN
				// little endian: no conversion needed.
				return WriteBuffer(ptr,ElementCount*sizeof(T));
			#else
				// big endian: the individual "<<" functions already convert endiannes
				for (size_t i=0;i<ElementCount;i++) (*this) << ptr[i];
			#endif
			}


			/** Copies a specified number of bytes from one stream to another. */
			size_t  CopyFrom(CStream* Source, size_t Count);

			/** Introduces a pure virtual method for moving to a specified position in the streamed resource.
			 *   he Origin parameter indicates how to interpret the Offset parameter. Origin should be one of the following values:
			 *	- sFromBeginning	(Default) Offset is from the beginning of the resource. Seek moves to the position Offset. Offset must be >= 0.
			 *	- sFromCurrent		Offset is from the current position in the resource. Seek moves to Position + Offset.
			 *	- sFromEnd			Offset is from the end of the resource. Offset must be <= 0 to indicate a number of bytes before the end of the file.
			 * \return Seek returns the new value of the Position property.
			 */
			virtual uint64_t Seek(long Offset, CStream::TSeekOrigin Origin = sFromBeginning) = 0;

			/** Returns the total amount of bytes in the stream.
			 */
			virtual uint64_t getTotalBytesCount() = 0;

			/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
			 */
			virtual uint64_t getPosition() =0;

			/** Writes an object to the stream.
			 */
			void WriteObject( const CSerializable *o );

			/** Reads an object from stream, its class determined at runtime, and returns a smart pointer to the object.
			 * \exception std::exception On I/O error or undefined class.
			 * \exception mrpt::utils::CExceptionEOF On an End-Of-File condition found at a correct place: an EOF that abruptly finishes in the middle of one object raises a plain std::exception instead.
			 */
			CSerializablePtr ReadObject();

			/** Reads an object from stream, where its class must be the same
			 *    as the supplied object, where the loaded object will be stored in.
			 * \exception std::exception On I/O error or different class found.
			 * \exception mrpt::utils::CExceptionEOF On an End-Of-File condition found at a correct place: an EOF that abruptly finishes in the middle of one object raises a plain std::exception instead.
			 */
			void ReadObject(CSerializable *existingObj);

			/** Write an object to a stream in the binary MRPT format. */
			CStream& operator << (const CSerializablePtr & pObj);
			/** Write an object to a stream in the binary MRPT format. */
			CStream& operator << (const CSerializable &obj);

			CStream& operator >> (CSerializablePtr &pObj);
			CStream& operator >> (CSerializable &obj);



			/** Writes a string to the stream in a textual form.
			  * \sa CStdOutStream
			  */
			virtual int printf(const char *fmt,...) MRPT_printf_format_check(2,3);  // The first argument (1) is "this" !!!

			/** Prints a vector in the format [A,B,C,...] using CStream::printf, and the fmt string for <b>each</b> vector element. */
			template <typename T>
			void printf_vector(const char *fmt, const std::vector<T> &V )
			{
				this->printf("[");
				size_t N = V.size();
				for (size_t i=0;i<N;i++)
				{
					this->printf(fmt,V[i]);
					if (i!=(N-1)) this->printf(",");
				}
				this->printf("]");
			}

			/** Send a message to the device.
			 *  Note that only the low byte from the "type" field will be used.
			 *
			 *  For frames of size < 255 the frame format is an array of bytes in this order:
			 *  \code
			 *  <START_FLAG> <HEADER> <LENGTH> <BODY> <END_FLAG>
			 *  	<START_FLAG> 	= 0x69
			 *  	<HEADER> 		= A header byte
			 *  	<LENGHT>		= Number of bytes of BODY
			 *  	<BODY>			= N x bytes
			 *  	<END_FLAG>		= 0X96
			 *  Total length 	= 	<LENGTH> + 4
			 *  \endcode
			 *
			 *  For frames of size > 255 the frame format is an array of bytes in this order:
			 *  \code
			 *  <START_FLAG> <HEADER> <HIBYTE(LENGTH)> <LOBYTE(LENGTH)> <BODY> <END_FLAG>
			 *  	<START_FLAG> 	= 0x79
			 *  	<HEADER> 		= A header byte
			 *  	<LENGHT>		= Number of bytes of BODY
			 *  	<BODY>			= N x bytes
			 *  	<END_FLAG>		= 0X96
			 *  Total length 	= 	<LENGTH> + 5
			 *  \endcode
			 *
			 * \exception std::exception On communication errors
			 */
			void  sendMessage( const utils::CMessage &msg);

			/** Tries to receive a message from the device.
			  * \exception std::exception On communication errors
			  * \returns True if successful, false if there is no new data from the device (but communications seem to work fine)
			  * \sa The frame format is described in sendMessage()
			  */
			bool  receiveMessage( utils::CMessage &msg );

			/** Reads from the stream until a '\n' character is found ('\r' characters are ignored).
			  * \return false on EOF or any other read error.
			  */
			bool getline(std::string &out_str);


		}; // End of class def.


	#define DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( T ) \
		CStream BASE_IMPEXP & operator<<(CStream&out, const T &a); \
		CStream BASE_IMPEXP & operator>>(CStream&in, T &a);

		// Definitions:
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( bool )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint8_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( int8_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint16_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( int16_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint32_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( int32_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint64_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( int64_t )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( float )
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( double )
#ifdef HAVE_LONG_DOUBLE
		DECLARE_CSTREAM_READ_WRITE_SIMPLE_TYPE( long double )
#endif

		// Why this shouldn't be templatized?: There's a more modern system
		// in MRPT that serializes any kind of vector<T>, deque<T>, etc... but
		// to keep COMPATIBILITY with old serialized objects we must preserve
		// the ones listed here:

		// Write --------------------
		CStream BASE_IMPEXP & operator << (CStream&s, const char *a);
		CStream BASE_IMPEXP & operator << (CStream&s, const std::string &str);

		CStream BASE_IMPEXP & operator << (CStream&s, const vector_float  &a);
		CStream BASE_IMPEXP & operator << (CStream&s, const vector_double &a);

		CStream BASE_IMPEXP & operator << (CStream&, const vector_int &a);
		CStream BASE_IMPEXP & operator << (CStream&, const vector_uint &a);
		CStream BASE_IMPEXP & operator << (CStream&, const vector_word &a);
		CStream BASE_IMPEXP & operator << (CStream&, const vector_signed_word &a);
		CStream BASE_IMPEXP & operator << (CStream&, const vector_long &a);
		CStream BASE_IMPEXP & operator << (CStream&, const vector_byte &a);
		CStream BASE_IMPEXP & operator << (CStream&, const vector_signed_byte &a);

		CStream BASE_IMPEXP & operator << (CStream&, const vector_bool &a);
		CStream BASE_IMPEXP & operator << (CStream&, const std::vector<std::string> &);

        #if MRPT_WORD_SIZE!=32  // If it's 32 bit, size_t <=> uint32_t
		CStream BASE_IMPEXP & operator << (CStream&, const std::vector<size_t> &a);
        #endif

		// Read --------------------
		CStream BASE_IMPEXP & operator>>(CStream&in, char *a);
		CStream BASE_IMPEXP & operator>>(CStream&in, std::string &str);

		CStream BASE_IMPEXP & operator>>(CStream&in, vector_double &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_float &a);

		CStream BASE_IMPEXP & operator>>(CStream&in, vector_int &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_uint &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_word &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_signed_word &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_long &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_byte &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_signed_byte &a);
		CStream BASE_IMPEXP & operator>>(CStream&in, vector_bool &a);

		CStream BASE_IMPEXP & operator>>(CStream&in, std::vector<std::string> &a);

		// For backward compatibility, since in MRPT<0.8.1 vector_XXX and std::vector<XXX> were exactly equivalent, now there're not.
		CStream BASE_IMPEXP & operator >> (CStream&s, std::vector<float>  &a);
		CStream BASE_IMPEXP & operator >> (CStream&s, std::vector<double> &a);
		CStream BASE_IMPEXP & operator << (CStream&s, const std::vector<float>  &a);
		CStream BASE_IMPEXP & operator << (CStream&s, const std::vector<double> &a);

        #if MRPT_WORD_SIZE!=32  // If it's 32 bit, size_t <=> uint32_t
		CStream BASE_IMPEXP & operator >> (CStream&s, std::vector<size_t> &a);
        #endif

	} // End of namespace
} // End of namespace

#endif
