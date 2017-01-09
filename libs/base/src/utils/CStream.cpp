/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/types_math.h> // CVector* types

#include <map>
#include <iostream>
#include <cstdarg>

#include "internal_class_registry.h"

// 8 bits:
#define SERIALIZATION_END_FLAG  0x88

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CStream::~CStream()
{

}

/*---------------------------------------------------------------
							ReadBuffer
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t  CStream::ReadBuffer(void *Buffer, size_t Count)
{
	ASSERT_(Buffer!=NULL)
	if (Count)
	{
		size_t		actuallyRead = Read(Buffer,Count);
		if ( !actuallyRead )
		{
			THROW_EXCEPTION("(EOF?) Cannot read requested number of bytes from stream" );
		}
		else
		{
			return actuallyRead;
		}
	}
	else return 0;
}

/*---------------------------------------------------------------
							WriteBuffer
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
void  CStream::WriteBuffer(const void *Buffer, size_t Count)
{
	ASSERT_(Buffer!=NULL)
	if (Count)
		if ( Count != Write(Buffer,Count) )
			THROW_EXCEPTION("Cannot write bytes to stream!" );
}


/*---------------------------------------------------------------
			Writes an elemental data type to stream.
 ---------------------------------------------------------------*/
#if MRPT_IS_BIG_ENDIAN
	// Big endian system: Convert into little-endian for streaming
	#define IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( T ) \
		CStream& utils::operator<<(mrpt::utils::CStream&out, const T &a) \
		{ \
			T b; \
			::memcpy(&b,&a,sizeof(b)); \
			mrpt::utils::reverseBytesInPlace(b); \
			out.WriteBuffer( (void*)&b, sizeof(b) ); \
			return out; \
		} \
		CStream& utils::operator>>(mrpt::utils::CStream&in, T &a) \
		{ \
			T b; \
			in.ReadBuffer( (void*)&b, sizeof(a) ); \
			mrpt::utils::reverseBytesInPlace(b); \
			::memcpy(&a,&b,sizeof(b)); \
			return in; \
		}
#else
	// Little endian system:
	#define IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( T ) \
		CStream& utils::operator<<(mrpt::utils::CStream&out, const T &a) \
		{ \
			out.WriteBuffer( (void*)&a, sizeof(a) ); \
			return out; \
		} \
		CStream& utils::operator>>(mrpt::utils::CStream&in, T &a) \
		{ \
			in.ReadBuffer( (void*)&a, sizeof(a) ); \
			return in; \
		}
#endif

IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( bool )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint8_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( int8_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint16_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( int16_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint32_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( int32_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( uint64_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( int64_t )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( float )
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( double )

#ifdef HAVE_LONG_DOUBLE
IMPLEMENT_CSTREAM_READ_WRITE_SIMPLE_TYPE( long double )
#endif


CStream& utils::operator << (mrpt::utils::CStream&out, const char *s)
{
	uint32_t l = (uint32_t)strlen(s);
	out << l;
	out.WriteBuffer( s, (int)l );
	return out;
}


CStream& utils::operator << (mrpt::utils::CStream&out,const  vector_bool &a)
{
	uint32_t n = (uint32_t) a.size();
	out << n;
	if (n)
	{
		vector_byte		b(n);
		vector_bool::const_iterator it;
		vector_byte::iterator it2;
		for (it=a.begin(),it2=b.begin();it!=a.end();++it,++it2)	*it2 = *it ? 1:0;
		out.WriteBuffer( (void*)&b[0], (int)(sizeof(b[0])*n) );
	}
	return out;
}

CStream& utils::operator << (mrpt::utils::CStream&out, const std::string &str)
{
	uint32_t n = (uint32_t) str.size();
	out << n;
	if (n)
		out.WriteBuffer( str.c_str(), n );
	return out;
}

/*---------------------------------------------------------------
			Writes an object to the stream.
 ---------------------------------------------------------------*/
void CStream::WriteObject(const  CSerializable *o )
{
    MRPT_START


	int		version;

	// First, the "classname".
	const char *className;
	if(o != NULL)
	{
		className = o->GetRuntimeClass()->className;
	}
	else
	{
		className = "nullptr";	
	}
	
	int8_t  classNamLen = strlen(className);
	int8_t  classNamLen_mod = classNamLen | 0x80;

	(*this) << classNamLen_mod;
	this->WriteBuffer( className, classNamLen);

	// Next, the version number:
	if(o != NULL)
	{
		o->writeToStream(*this, &version);
		ASSERT_(version>=0 && version<255);

		int8_t	actualVersion = int8_t(version);
		(*this) << actualVersion;

		// Next, the object data.
		o->writeToStream(*this, NULL);
	}

	// In MRPT 0.5.5 a end flag is introduced:
	static const uint8_t    endFlag = SERIALIZATION_END_FLAG;
	(*this) << endFlag;

    MRPT_END
}


CStream& CStream::operator << (const CSerializablePtr & pObj)
{
	WriteObject(pObj.pointer());
	return *this;
}

/** Write an object to a stream in the binary MRPT format. */
CStream& CStream::operator << (const CSerializable &obj)
{
	WriteObject(&obj);
	return *this;
}

CStream& CStream::operator >> (CSerializablePtr &pObj)
{
	pObj = ReadObject();
	return *this;
}

CStream& CStream::operator >> (CSerializable &obj)
{
	ReadObject( &obj );
	return *this;
}


/*---------------------------------------------------------------
			Reads an elemental data type from the stream.
 ---------------------------------------------------------------*/

namespace mrpt
{
	namespace utils
	{
		// For backward compatibility, since in MRPT<0.8.1 vector_XXX and std::vector<XXX> were exactly equivalent, now there're not.
		namespace detail {
			template <typename VEC> inline CStream& writeStdVectorToStream(mrpt::utils::CStream&s, const VEC &v) {
				const uint32_t n = static_cast<uint32_t>(v.size());
				s << n; if (n) s.WriteBufferFixEndianness( &v[0],n );
				return s;
			}
			template <typename VEC> inline CStream& readStdVectorToStream(mrpt::utils::CStream&s, VEC &v) {
				uint32_t n; s >> n;
				v.resize(n); if (n) s.ReadBufferFixEndianness( &v[0], n );
				return s;
			}
		}
	}
}

// Write:
CStream& utils::operator << (mrpt::utils::CStream&s, const std::vector<float>  &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const std::vector<double> &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_int &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_uint &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_word &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_signed_word &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_long &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_byte  &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const vector_signed_byte  &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator << (mrpt::utils::CStream&s, const mrpt::math::CVectorFloat  &v)
{
	const uint32_t n = static_cast<uint32_t>(v.size());
	s << n; if (n) s.WriteBufferFixEndianness( &v[0],n );
	return s;
}
CStream& utils::operator << (mrpt::utils::CStream&s, const mrpt::math::CVectorDouble  &v)
{
	const uint32_t n = static_cast<uint32_t>(v.size());
	s << n; if (n) s.WriteBufferFixEndianness( &v[0],n );
	return s;
}

// Read:
CStream& utils::operator >> (mrpt::utils::CStream&s, std::vector<float>  &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, std::vector<double> &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_int &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_uint &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_word &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_signed_word &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_long &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_byte &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, vector_signed_byte &a) { return detail::readStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, mrpt::math::CVectorFloat &v)
{
	uint32_t n; s >> n;
	v.resize(n);
	if (n) s.ReadBufferFixEndianness( &v[0], n );
	return s;
}
CStream& utils::operator >> (mrpt::utils::CStream&s, mrpt::math::CVectorDouble &v)
{
	uint32_t n; s >> n;
	v.resize(n);
	if (n) s.ReadBufferFixEndianness( &v[0], n );
	return s;
}

#if MRPT_WORD_SIZE!=32  // If it's 32 bit, size_t <=> uint32_t
CStream& utils::operator << (mrpt::utils::CStream&s, const std::vector<size_t> &a) { return detail::writeStdVectorToStream(s,a); }
CStream& utils::operator >> (mrpt::utils::CStream&s, std::vector<size_t> &a) { return detail::readStdVectorToStream(s,a); }
#endif

CStream& utils::operator>>(mrpt::utils::CStream&in, vector_bool &a)
{
	uint32_t   n;
	in >> n;
	a.resize(n);
	if (n)
	{
		vector_byte	b(n);
		in.ReadBuffer( (void*)&b[0], sizeof(b[0])*n);
		vector_byte::iterator it2;
		vector_bool::iterator it;
		for (it=a.begin(),it2=b.begin();it!=a.end();++it,++it2) *it = (*it2!=0);
	}
	return in;
}

CStream& utils::operator>>(mrpt::utils::CStream&in, std::string &str)
{
	uint32_t   n;
	in >> n;
	str.resize(n);
	if (n)
		in.ReadBuffer( (void*)&str[0], n );
	return in;
}


CStream& utils::operator>>(mrpt::utils::CStream&in, char *s)
{
	ASSERT_(s!=NULL)
	uint32_t  l;
	in >>l;
	if (l)
		in.ReadBuffer( s, l );
	s[l]='\0';
	return in;
}


//#define CSTREAM_VERBOSE     1
#define CSTREAM_VERBOSE     0

template <bool EXISTING_OBJ>
void CStream::internal_ReadObject(CSerializablePtr &newObj,CSerializable *existingObj)
{
	// Automatically register all classes when the first one is registered.
	registerAllPendingClasses();

	// First, read the class name:
	uint8_t lengthReadClassName = 255;
	bool    isOldFormat=false;   // < MRPT 0.5.5
	char    readClassName[260];
	readClassName[0] = 0;

	try
	{
		// First, read the class name: (exception is raised here if ZERO bytes read -> possibly an EOF)
		if (sizeof(lengthReadClassName) != ReadBuffer( (void*)&lengthReadClassName, sizeof(lengthReadClassName) ) )
			THROW_EXCEPTION("Cannot read object header from stream! (EOF?)");

		// Is in old format (< MRPT 0.5.5)?
		if (! (lengthReadClassName & 0x80 ))
		{
			isOldFormat = true;
			uint8_t buf[3];
			if (3 != ReadBuffer( buf, 3 ) ) THROW_EXCEPTION("Cannot read object header from stream! (EOF?)");
			if (buf[0] || buf[1] || buf[2]) THROW_EXCEPTION("Expecting 0x00 00 00 while parsing old streaming header (Perhaps it's a gz-compressed stream? Use a GZ-stream for reading)");
		}

		// Remove MSB:
		lengthReadClassName &= 0x7F;

		// Sensible class name size?
		if (lengthReadClassName>120)
			THROW_EXCEPTION("Class name has more than 120 chars. This probably means a corrupted binary stream.");

		if (((size_t)lengthReadClassName)!=ReadBuffer( readClassName, lengthReadClassName ))
			THROW_EXCEPTION("Cannot read object class name from stream!");

		readClassName[lengthReadClassName]='\0';

		// Pass to string class:
		const std::string strClassName(readClassName);

		// Next, the version number:
		int8_t version;
		if (isOldFormat)
		{
			int32_t  version_old;
			// Handle big endian right:
			if (sizeof(version_old)!=ReadBufferFixEndianness( &version_old, 1 /*element count*/ ))
				THROW_EXCEPTION("Cannot read object streaming version from stream!");
			ASSERT_(version_old>=0 && version_old<255);
			version = int8_t(version_old);
		}
		else if (strClassName != "nullptr" && sizeof(version)!=ReadBuffer( (void*)&version, sizeof(version) ))
		{
				THROW_EXCEPTION("Cannot read object streaming version from stream!");
		}

		// In MRPT 0.5.5 an end flag was introduced:
#if CSTREAM_VERBOSE
		cerr << "[CStream::ReadObject] readClassName:" << strClassName << " version: " << version <<  endl;
#endif

		CSerializable* obj = NULL;
		if (EXISTING_OBJ)
		{	// (Existing object)
			// Now, compare to existing class:
			if(strClassName != "nullptr")
			{
				ASSERT_(existingObj)
				const TRuntimeClassId	*id  = existingObj->GetRuntimeClass();
				const TRuntimeClassId	*id2 = findRegisteredClass(strClassName);
				if (!id2) THROW_EXCEPTION_CUSTOM_MSG1("Stored object has class '%s' which is not registered!",strClassName.c_str());
				if ( id!=id2 ) THROW_EXCEPTION(format("Stored class does not match with existing object!!:\n Stored: %s\n Expected: %s", id2->className,id->className ));
				// It matches, OK
				obj = existingObj;
			}
		}
		else if (strClassName != "nullptr")
		{	// (New object)
			// Get the mapping to the "TRuntimeClassId*" in the registered classes table:
			const TRuntimeClassId *classId = findRegisteredClass( strClassName );
			if (!classId)
			{
				const std::string msg = format("Class '%s' is not registered! Have you called mrpt::registerClass(CLASS)?",readClassName);
				std::cerr << "CStream::ReadObject(): " << msg << std::endl;
				THROW_EXCEPTION(msg)
			}
			obj = static_cast<CSerializable*>(classId->createObject());
			newObj = CSerializablePtr(obj);
		}

		if(strClassName != "nullptr")
		{
			// Go on, read it:
			obj->readFromStream( *this, (int)version );
		}
		// Check end flag (introduced in MRPT 0.5.5)
		if (!isOldFormat)
		{
			uint8_t	endFlag;
			if (sizeof(endFlag)!=ReadBuffer( (void*)&endFlag, sizeof(endFlag) )) THROW_EXCEPTION("Cannot read object streaming version from stream!");
			if (endFlag!=SERIALIZATION_END_FLAG) THROW_EXCEPTION_CUSTOM_MSG1("end-flag missing: There is a bug in the deserialization method of class: '%s'",strClassName.c_str());
		}
		ASSERT_(!EXISTING_OBJ || strClassName != "nullptr");
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
	catch(std::exception &e)
	{
		if (lengthReadClassName==255) {
			THROW_TYPED_EXCEPTION("Cannot read object due to EOF", CExceptionEOF);
		}
		else {
			THROW_STACKED_EXCEPTION_CUSTOM_MSG2(e,"Exception while parsing typed object '%s' from stream!\n",readClassName);
		}
	}
	catch (...) {
		THROW_EXCEPTION("Unexpected runtime error!");
	}
} // end method


/*---------------------------------------------------------------
	Reads an object from stream, where its class is determined
	at runtime.
	  exception std::exception On I/O error or undefined class.
 ---------------------------------------------------------------*/
CSerializablePtr CStream::ReadObject()
{
	CSerializablePtr ret;
	internal_ReadObject<false>(ret, NULL);
	return ret;
}

/*---------------------------------------------------------------
	Reads an object from stream, where its class is determined
	by an existing object
	  exception std::exception On I/O error or undefined class.
 ---------------------------------------------------------------*/
void CStream::ReadObject(CSerializable *existingObj)
{
	CSerializablePtr dummy;
	internal_ReadObject<true>(dummy,existingObj);
}

/*---------------------------------------------------------------
			Writes an elemental data type to stream.
 ---------------------------------------------------------------*/
int CStream::printf(const char *fmt, ... )
{
	MRPT_START

	if (!fmt) throw std::runtime_error("fmt in CStream::printf cannot be NULL");

	int   result = -1, length = 1024;
	vector<char> buffer;
	while (result == -1)
	{
		buffer.resize(length + 10);

		va_list args;  // This must be done WITHIN the loop
		va_start(args,fmt);
		result = os::vsnprintf(&buffer[0], length, fmt, args);
		va_end(args);

		// Truncated?
		if (result>=length) result=-1;
		length*=2;
	}

	size_t l = strlen(&buffer[0]);
	WriteBuffer( &buffer[0], (int)l );

	return result;

	MRPT_END
}

CStream &utils::operator<<(mrpt::utils::CStream &s,const std::vector<std::string> &vec)	{
	uint32_t N=static_cast<uint32_t>(vec.size());
	s<<N;
	for (size_t i=0;i<N;i++) s<<vec[i];
	return s;
}

CStream &utils::operator>>(mrpt::utils::CStream &s,std::vector<std::string> &vec)	{
	uint32_t N;
	s>>N;
	vec.resize(N);
	for (size_t i=0;i<N;i++) s>>vec[i];
	return s;
}


/*-------------------------------------------------------------
					sendMessage
-------------------------------------------------------------*/
void  CStream::sendMessage(const utils::CMessage &msg)
{
	MRPT_START

	unsigned char	buf[0x10100];
	unsigned int	nBytesTx = 0;

	const bool msg_format_is_tiny = msg.content.size() < 256;

	// Build frame -------------------------------------
	buf[nBytesTx++] = msg_format_is_tiny ? 0x69 : 0x79;
	buf[nBytesTx++] = (unsigned char)(msg.type);

	if (msg_format_is_tiny) {
		buf[nBytesTx++] = (unsigned char)msg.content.size();
	} else {
		buf[nBytesTx++] = msg.content.size() & 0xff;      // lo
		buf[nBytesTx++] = (msg.content.size()>>8) & 0xff; // hi
	}

	if (!msg.content.empty())
		memcpy(	buf+nBytesTx, &msg.content[0], msg.content.size() );
	nBytesTx += (unsigned char)msg.content.size();
	buf[nBytesTx++] = 0x96;

	// Send buffer -------------------------------------
	WriteBuffer(buf,nBytesTx);  // Exceptions will be raised on errors here

	MRPT_END
}

/*-------------------------------------------------------------
					receiveMessage
-------------------------------------------------------------*/
bool  CStream::receiveMessage( utils::CMessage &msg )
{
	MRPT_START
	std::vector<unsigned char> buf(66000);
	unsigned int		nBytesInFrame=0;
	unsigned long		nBytesToRx=0;
	unsigned char		tries = 2;
	unsigned int		payload_len = 0;
	unsigned int		expectedLen = 0;

	for (;;)
	{
		if (nBytesInFrame<4)
			nBytesToRx = 1;
		else
		{
			if (buf[0] == 0x69) 
			{
				payload_len = buf[2];
				expectedLen = payload_len + 4;
			}
			else if( buf[0] == 0x79 )
			{
				payload_len = MAKEWORD16B(buf[3] /*low*/, buf[2] /*hi*/); // Length of the content
				expectedLen = payload_len + 5;
			}
			nBytesToRx = expectedLen - nBytesInFrame;
		} // end else

		unsigned long nBytesRx = 0; 
		try {
			nBytesRx = ReadBufferImmediate(&buf[nBytesInFrame], nBytesToRx);
		}
		catch (...) {
		}

		// No more data! (read timeout is already included in the call to "Read")
		if (!nBytesRx)
			return false;

		if (!nBytesInFrame && buf[0]!=0x69 && buf[0]!=0x79 )
		{
			// Start flag is invalid:
			if (!tries--) return false;
		}
		else
		{
			// Is a new byte for the frame:
			nBytesInFrame += nBytesRx;

			if (nBytesInFrame == expectedLen )
			{
				// Frame complete
				// check for frame be ok:

				// End flag?
				if (buf[nBytesInFrame-1]!=0x96)
				{
					// Error in frame!
					nBytesInFrame=0;
					return false;
				}
				else
				{
					// copy out data:
					msg.type = buf[1];
					if( buf[0] == 0x69 )
					{
						msg.content.resize(payload_len);
						if (!msg.content.empty())
							memcpy( &msg.content[0], &buf[3], payload_len);
					} // end if
					if ( buf[0] == 0x79 )
					{
						msg.content.resize(payload_len);
						if (!msg.content.empty())
							memcpy( &msg.content[0], &buf[4], payload_len);
					} // end if
					return true;
				}
			}
		}
	}
	MRPT_END
}

/*-------------------------------------------------------------
Reads from the stream until a '\n' character is found ('\r' characters are ignored).
return false on EOF or any other read error.
-------------------------------------------------------------*/
bool CStream::getline(std::string &out_str)
{
	out_str.clear();
	try
	{
		for (;;)
		{
			size_t N = out_str.size();
			out_str.resize(N+1);
			if (! Read(&out_str[N], 1) )
				return false;

			// New char read:
			if (out_str[N]=='\r')
			{
				out_str.resize(N); // Ignore.
			}
			else if (out_str[N]=='\n')
			{
				out_str.resize(N); // End of line!
				return true; // Ok.
			}
		}
	}
	catch(...)
	{	// Any read error:
		return false;
	}
}
