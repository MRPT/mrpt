/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include "zlib.h"

#include <mrpt/config.h>

// For named pipes:
#ifdef MRPT_OS_WINDOWS
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>
#endif


#include <mrpt/compress/zip.h>
#include <mrpt/system/os.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileInputStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::compress::zip;

/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void  mrpt::compress::zip::compress(
	void						*inData,
	size_t						inDataSize,
	std::vector<unsigned char>	&outData)
{
	int ret=0;
	MRPT_START

	unsigned long	resSize;

	outData.resize(inDataSize+inDataSize/1000+50);
	resSize = (unsigned long)outData.size();
	ret = ::compress(
		&outData[0],
		&resSize,
		(unsigned char*)inData,
		(unsigned long)inDataSize
		);
	ASSERT_(ret==Z_OK);

	outData.resize(resSize);

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}

/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void  mrpt::compress::zip::compress(
	const std::vector<unsigned char>	&inData,
	std::vector<unsigned char>			&outData)
{
	int ret=0;
	MRPT_START

	unsigned long	resSize;

	outData.resize(inData.size()+inData.size()/1000+50);
	resSize = (unsigned long)outData.size();
	ret = ::compress(
		&outData[0],
		&resSize,
		&inData[0],
		(unsigned long)inData.size()
		);
	ASSERT_(ret==Z_OK);

	outData.resize(resSize);

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}

/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void  mrpt::compress::zip::compress(
	void						*inData,
	size_t						inDataSize,
	CStream						&out)
{
	int ret=0;
	MRPT_START

	unsigned long	resSize;
	std::vector<unsigned char>	outData;

	outData.resize(inDataSize+inDataSize/1000+50);
	resSize = (unsigned long)outData.size();

	ret = ::compress(
		&outData[0],
		&resSize,
		(unsigned char*)inData,
		(unsigned long)inDataSize
		);
	ASSERT_(ret==Z_OK);

	outData.resize(resSize);

	// Write to stream:
	out.WriteBuffer( &outData[0], resSize );

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}


/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void  mrpt::compress::zip::compress(
	const std::vector<unsigned char>	&inData,
	CStream						&out)
{
	int ret=0;
	MRPT_START

	unsigned long	resSize;
	std::vector<unsigned char>	outData;
	outData.resize(inData.size()+inData.size()/1000+50);
	resSize = (unsigned long)outData.size();

	ret = ::compress(
		&outData[0],
		&resSize,
		&inData[0],
		(unsigned long)inData.size()
		);

	ASSERT_(ret==Z_OK);

	outData.resize(resSize);

	// Write to stream:
	out.WriteBuffer( &outData[0], resSize );

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}

/*---------------------------------------------------------------
						decompress
---------------------------------------------------------------*/
void  mrpt::compress::zip::decompress(
	void						*inData,
	size_t						inDataSize,
	std::vector<unsigned char>	&outData,
	size_t						outDataEstimatedSize)
{
	int ret=0;
	MRPT_START

	outData.resize( outDataEstimatedSize );
	unsigned long	actualOutSize;

	ret = ::uncompress(
		&outData[0],
		&actualOutSize,
		(unsigned char*)inData,
		(unsigned long)inDataSize
		);

	ASSERT_(ret==Z_OK);

	outData.resize(actualOutSize);

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}

/*---------------------------------------------------------------
						decompress
---------------------------------------------------------------*/
void  mrpt::compress::zip::decompress(
	void						*inData,
	size_t						inDataSize,
	void						*outData,
	size_t						outDataBufferSize,
	size_t						&outDataActualSize)
{
	int ret=0;
	MRPT_START

	unsigned long	actualOutSize = (unsigned long)outDataBufferSize;

	ret = ::uncompress(
		(unsigned char*)outData,
		&actualOutSize,
		(unsigned char*)inData,
		(unsigned long)inDataSize
		);

	ASSERT_(ret==Z_OK);

	outDataActualSize = actualOutSize;

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}

/*---------------------------------------------------------------
						decompress
---------------------------------------------------------------*/
void  mrpt::compress::zip::decompress(
	CStream						&inStream,
	size_t						inDataSize,
	void						*outData,
	size_t						outDataBufferSize,
	size_t						&outDataActualSize)
{
	int ret=0;
	MRPT_START

	unsigned long	actualOutSize = (unsigned long)outDataBufferSize;
	std::vector<unsigned char>		inData;

	inData.resize(inDataSize);
	inStream.ReadBuffer( &inData[0], inDataSize );

	ret = ::uncompress(
		(unsigned char*)outData,
		&actualOutSize,
		&inData[0],
		(unsigned long)inDataSize
		);

	ASSERT_(ret==Z_OK);

	outDataActualSize = actualOutSize;

	MRPT_END_WITH_CLEAN_UP( \
		printf("[zlib] Error code=%i\n",ret);\
		);
}


/*---------------------------------------------------------------
					decompress_gz_file
---------------------------------------------------------------*/
bool mrpt::compress::zip::decompress_gz_file(const std::string &file_path, vector_byte & buffer)
{
	CFileGZInputStream	iss(file_path);
	if (!iss.fileOpenCorrectly())
		return false;

	buffer.clear();

	const int bytes2read = 1 << 20;
	int	act_read;
	size_t	total_bytes = 0;

	buffer.reserve( iss.getTotalBytesCount() );

	do
	{
		buffer.resize( 1000+bytes2read + buffer.size() );
		act_read = iss.ReadBuffer( &buffer[total_bytes], bytes2read );
		total_bytes += act_read;
	} while (act_read==bytes2read);

	buffer.resize( total_bytes );
	return true;
}

/*---------------------------------------------------------------
					compress_gz_file
---------------------------------------------------------------*/
bool mrpt::compress::zip::compress_gz_file(
	const std::string &file_path,
	const vector_byte &buffer,
	const int compress_level
	)
{
#if MRPT_HAS_GZ_STREAMS
	CFileGZOutputStream		oss;
	oss.open(file_path,compress_level);
	if (!oss.fileOpenCorrectly())
		return false;

	if (buffer.size())
	{
		try
		{
			oss.WriteBuffer(&buffer[0], buffer.size());
			return true;
		}
		catch(...) { return false; }
	}
	else
	{
		return true;
	}
#else
	THROW_EXCEPTION("MRPT has been compiled with MRPT_HAS_GZ_STREAMS=0")
#endif
}


/*---------------------------------------------------------------
					compress_gz_data_block
---------------------------------------------------------------*/
bool mrpt::compress::zip::compress_gz_data_block(
	const vector_byte &in_data,
	vector_byte &out_gz_data,
	const int compress_level
	)
{
	out_gz_data.clear();
	if (in_data.empty())
		return true;

#if MRPT_HAS_GZ_STREAMS

	const unsigned int nPipeName = static_cast<unsigned int>(mrpt::system::now());
	std::string   pipe_file_name;

	// Create an anonymous pipe for writing the data to:
	#ifdef MRPT_OS_WINDOWS
		// Windows:
		pipe_file_name = format("\\\\.\\pipe\\mrpt_compress_gz_data_block_%u",nPipeName);

		HANDLE hPipe = CreateNamedPipeA(
			pipe_file_name.c_str(),
			PIPE_ACCESS_DUPLEX | 0x00080000 /* FILE_FLAG_FIRST_PIPE_INSTANCE */,
			PIPE_TYPE_BYTE,
			PIPE_UNLIMITED_INSTANCES,
			in_data.size()+1000,
			in_data.size()+1000,
			0,
			NULL);
		if (hPipe==INVALID_HANDLE_VALUE) THROW_EXCEPTION("Error creating named pipe for gz-file compression");
	#else
		// Unix:
		pipe_file_name = format("/tmp/mrpt_compress_gz_data_block_%u",nPipeName);
	#endif


	bool retVal = false;
	try
	{
		// Write as gz
		{
			gzFile f = gzopen(pipe_file_name.c_str(),format("wb%i",compress_level).c_str() );
			if (f)
			{
				retVal = (int)in_data.size() == gzwrite(f,&in_data[0], in_data.size());
				gzclose(f);
			}
			else
			{
				std::cerr << "[compress_gz_data_block] Error writing to pipe: " << pipe_file_name << std::endl;
			}
		}
		// Read it all:
		if (retVal)
		{
		#ifdef MRPT_OS_WINDOWS
			// Read (windows)
			const size_t N = GetFileSize(hPipe,NULL);
			if (N)
			{
				out_gz_data.resize(N);
				DWORD nRead;
				SetFilePointer(hPipe,0,NULL, FILE_BEGIN);
				if (N)
				{
					ReadFile(hPipe,&out_gz_data[0],N,&nRead,NULL);
					retVal= nRead == N;
				}
			}
			else
			{
				retVal=false;
			}
		#else
			// Read (Unix)
			{
				CFileInputStream	iss;
				if (iss.open(pipe_file_name))
				{
					const size_t M = iss.getTotalBytesCount();
					out_gz_data.resize(M);
					if (M)
						retVal = M==iss.ReadBuffer(&out_gz_data[0],M);
				}
			}
		#endif
		}
	}
	catch(...)
	{
		retVal = false;
	}

	// Close the pipe:
	#ifdef MRPT_OS_WINDOWS
		CloseHandle(hPipe);
	#else
		remove(pipe_file_name.c_str());
	#endif

	return retVal;
#else
	THROW_EXCEPTION("MRPT has been compiled with MRPT_HAS_GZ_STREAMS=0")
#endif
}


/*---------------------------------------------------------------
					decompress_gz_data_block
---------------------------------------------------------------*/
bool mrpt::compress::zip::decompress_gz_data_block(
	const vector_byte &in_gz_data,
	vector_byte &out_data)
{
	out_data.clear();
	if (in_gz_data.empty()) return true;

	// JL: I tried to do this with pipes but had no luck... :-(

	const std::string  tmp_file_name = mrpt::system::getTempFileName();
	if (!mrpt::system::vectorToBinaryFile(in_gz_data,tmp_file_name)) return false;
	bool retVal = mrpt::compress::zip::decompress_gz_file(tmp_file_name,out_data);

	remove(tmp_file_name.c_str());  // Delete tmp file

	return retVal;
}


