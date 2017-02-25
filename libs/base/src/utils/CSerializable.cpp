/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;

#include <cstdio>


IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CSerializable, CObject, mrpt::utils)


/* -----------------------------------------------------------------------
	Used to pass MRPT objects into a CORBA-like object,
		a string. See doc about "Integration with BABEL".
   ----------------------------------------------------------------------- */
std::string utils::ObjectToString(const CSerializable *o)
{
	CMemoryStream				tmp,tmpCoded;
	std::string					str;

	try
	{
		tmp.WriteObject(o);
		size_t n = tmp.getTotalBytesCount();

		// Scan the string to code it:
		// ----------------------------------
		size_t lastIdx = 0;
		unsigned char	*data = (unsigned char*)tmp.getRawBufferData();
		for (size_t i=0;i<n;i++)
		{
			unsigned char c = data[i];
			// Search for first "0x00" byte:
			if ( c == 0x01 || !c )
			{
				// Copy all till now:
				tmpCoded.WriteBuffer( &data[lastIdx], i - lastIdx);
				lastIdx = i+1;

				// And code:
				//   0x01 --> 0x01 0x01
				//   0x00 --> 0x01 0x02
				unsigned char dumm[2];
				dumm[0] = 0x01;
				if (c)
						dumm[1] = 0x01;
				else	dumm[1] = 0x02;

				// Append to coded stream:
				tmpCoded.WriteBuffer( dumm, 2);
			}
		} // end for i

		// Copy the rest:
		if ( n!=lastIdx)
			tmpCoded.WriteBuffer( &data[lastIdx], n - lastIdx );

		// Copy to string object:
		n = tmpCoded.getTotalBytesCount();
		str.resize(n);
		memcpy(&str[0],tmpCoded.getRawBufferData(),n);
		return str;
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
	catch(std::exception &e)
	{
	    fprintf(stderr, "[ObjectToString] Exception: %s\n", e.what());
		return "";
	}
	catch(...)
	{
	    fprintf(stderr, "[ObjectToString] Unknown exception\n");
		return "";
	}
}


/* -----------------------------------------------------------------------
	Used to pass CORBA-like object into a MRPT object.
		See doc about "Integration with BABEL".
   ----------------------------------------------------------------------- */
void utils::StringToObject(const std::string &str, CSerializablePtr &obj)
{
	MRPT_START

	obj.clear_unique();
	if (str.empty()) return;

	CMemoryStream	tmp;
	size_t			n;
	size_t			i,lastIdx;

	obj.clear_unique();

	n = str.size();

	// Scan the string to decode it:
	// ----------------------------------
	lastIdx = 0;
	const char *data = str.c_str();
	unsigned char c;
	for (i=0;i<n && (c=data[i])!=0;i++)
	{
		// Search for first "0x01" byte:
		if ( c == 0x01 )
		{
			// Copy all till now:
			tmp.WriteBuffer( &data[lastIdx], i - lastIdx + 1 );
			i+=1; // +1 from "for" loop
			lastIdx = i+1;

			// And decode:
			//   0x01 0x01 --> 0x01
			//   0x01 0x02 --> 0x00
			if (data[i]==0x01)
					((unsigned char*)tmp.getRawBufferData())[tmp.getTotalBytesCount()-1] = (unsigned char)0x01;
			else 	((unsigned char*)tmp.getRawBufferData())[tmp.getTotalBytesCount()-1] = (unsigned char)0x00;
		}
	} // end for i

	// Copy the rest:
	if ( n!=lastIdx )
		tmp.WriteBuffer( &data[lastIdx], n - lastIdx );

	// And the '\0' char:
	char dummy = '\0';
	tmp.WriteBuffer( &dummy, sizeof(char) );

	tmp.Seek(0,CStream::sFromBeginning);
	obj = tmp.ReadObject();

	MRPT_END

}


/* -----------------------------------------------------------------------
				ObjectToOctetVector
   ----------------------------------------------------------------------- */
void utils::ObjectToOctetVector(const CSerializable *o, vector_byte & out_vector)
{
	try
	{
		CMemoryStream	tmp;
		tmp.WriteObject(o);

		size_t N = tmp.getTotalBytesCount();
		out_vector.resize(N);
		if (N)
		{
			os::memcpy( &out_vector[0],N,tmp.getRawBufferData(), N );
		}
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
	catch(std::exception &e)
	{
	    fprintf(stderr, "[ObjectToOctetVector] Exception: %s\n",e.what());
	}
	catch(...)
	{
		fprintf(stderr, "[ObjectToOctetVector] Unknown exception\n");
	}
}

/* -----------------------------------------------------------------------
				OctetVectorToObject
   ----------------------------------------------------------------------- */
void utils::OctetVectorToObject(const vector_byte & in_data, CSerializablePtr &obj)
{
	try
	{
		obj.clear_unique();

		if (in_data.empty()) return;

		CMemoryStream	tmp;
		tmp.assignMemoryNotOwn(&in_data[0], in_data.size());
		obj = tmp.ReadObject();
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
	catch(std::exception &e)
	{
	    fprintf(stderr, "[OctetVectorToObject] Exception: %s\n",e.what());
	}
	catch(...)
	{
		fprintf(stderr, "[OctetVectorToObject] Unknown exception\n");
	}
}

/* -----------------------------------------------------------------------
				ObjectToRawString
   ----------------------------------------------------------------------- */
void utils::ObjectToRawString(const CSerializable *o, std::string & out_vector)
{
	try
	{
		CMemoryStream	tmp;
		tmp.WriteObject(o);

		size_t N = tmp.getTotalBytesCount();
		out_vector.resize(N);
		if (N)
		{
			os::memcpy( &out_vector[0],N,tmp.getRawBufferData(), N );
		}
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
	catch(std::exception &e)
	{
	    fprintf(stderr, "[ObjectToRawString] Exception: %s\n",e.what());
	}
	catch(...)
	{
		fprintf(stderr, "[ObjectToRawString] Unknown exception\n");
	}
}

/* -----------------------------------------------------------------------
				RawStringToObject
   ----------------------------------------------------------------------- */
void utils::RawStringToObject(const std::string & in_data, CSerializablePtr &obj)
{
	try
	{
		obj.clear_unique();

		if (in_data.empty()) return;

		CMemoryStream	tmp;
		tmp.assignMemoryNotOwn(&in_data[0], in_data.size());
		obj = tmp.ReadObject();
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
	catch(std::exception &e)
	{
		fprintf(stderr, "[RawStringToObject] Exception: %s\n",e.what());
	}
	catch(...)
	{
		fprintf(stderr, "[RawStringToObject] Unknown exception\n");
	}
}


