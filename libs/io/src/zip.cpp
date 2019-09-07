/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include "zlib.h"

// For named pipes:
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <mrpt/core/exceptions.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/io/zip.h>

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/system/filesystem.h>
#include <chrono>
#include <iostream>

using namespace mrpt;
using namespace mrpt::io;
using namespace mrpt::io::zip;

void mrpt::io::zip::compress(
	void* inData, size_t inDataSize, std::vector<unsigned char>& outData)
{
	int ret = 0;
	MRPT_START

	unsigned long resSize;

	outData.resize(inDataSize + inDataSize / 1000 + 50);
	resSize = (unsigned long)outData.size();
	ret = ::compress(
		&outData[0], &resSize, static_cast<unsigned char*>(inData),
		(unsigned long)inDataSize);
	ASSERT_(ret == Z_OK);

	outData.resize(resSize);

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void mrpt::io::zip::compress(
	const std::vector<unsigned char>& inData,
	std::vector<unsigned char>& outData)
{
	int ret = 0;
	MRPT_START

	unsigned long resSize;

	outData.resize(inData.size() + inData.size() / 1000 + 50);
	resSize = (unsigned long)outData.size();
	ret = ::compress(
		&outData[0], &resSize, &inData[0], (unsigned long)inData.size());
	ASSERT_(ret == Z_OK);

	outData.resize(resSize);

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void mrpt::io::zip::compress(void* inData, size_t inDataSize, CStream& out)
{
	int ret = 0;
	MRPT_START

	unsigned long resSize;
	std::vector<unsigned char> outData;

	outData.resize(inDataSize + inDataSize / 1000 + 50);
	resSize = (unsigned long)outData.size();

	ret = ::compress(
		&outData[0], &resSize, reinterpret_cast<unsigned char*>(inData),
		(unsigned long)inDataSize);
	ASSERT_(ret == Z_OK);

	outData.resize(resSize);

	// Write to stream:
	out.Write(&outData[0], resSize);

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
						compress
---------------------------------------------------------------*/
void mrpt::io::zip::compress(
	const std::vector<unsigned char>& inData, CStream& out)
{
	int ret = 0;
	MRPT_START

	unsigned long resSize;
	std::vector<unsigned char> outData;
	outData.resize(inData.size() + inData.size() / 1000 + 50);
	resSize = (unsigned long)outData.size();

	ret = ::compress(
		&outData[0], &resSize, &inData[0], (unsigned long)inData.size());

	ASSERT_(ret == Z_OK);

	outData.resize(resSize);

	// Write to stream:
	out.Write(&outData[0], resSize);

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
						decompress
---------------------------------------------------------------*/
void mrpt::io::zip::decompress(
	void* inData, size_t inDataSize, std::vector<unsigned char>& outData,
	size_t outDataEstimatedSize)
{
	int ret = 0;
	MRPT_START

	outData.resize(outDataEstimatedSize);
	unsigned long actualOutSize;

	ret = ::uncompress(
		&outData[0], &actualOutSize, (unsigned char*)inData,
		(unsigned long)inDataSize);

	ASSERT_(ret == Z_OK);

	outData.resize(actualOutSize);

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
						decompress
---------------------------------------------------------------*/
void mrpt::io::zip::decompress(
	void* inData, size_t inDataSize, void* outData, size_t outDataBufferSize,
	size_t& outDataActualSize)
{
	int ret = 0;
	MRPT_START

	auto actualOutSize = (unsigned long)outDataBufferSize;

	ret = ::uncompress(
		(unsigned char*)outData, &actualOutSize, (unsigned char*)inData,
		(unsigned long)inDataSize);

	ASSERT_(ret == Z_OK);

	outDataActualSize = actualOutSize;

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
						decompress
---------------------------------------------------------------*/
void mrpt::io::zip::decompress(
	CStream& inStream, size_t inDataSize, void* outData,
	size_t outDataBufferSize, size_t& outDataActualSize)
{
	int ret = 0;
	MRPT_START

	auto actualOutSize = (unsigned long)outDataBufferSize;
	std::vector<unsigned char> inData;

	inData.resize(inDataSize);
	inStream.Read(&inData[0], inDataSize);

	ret = ::uncompress(
		(unsigned char*)outData, &actualOutSize, &inData[0],
		(unsigned long)inDataSize);

	ASSERT_(ret == Z_OK);

	outDataActualSize = actualOutSize;

	MRPT_END_WITH_CLEAN_UP(printf("[zlib] Error code=%i\n", ret););
}

/*---------------------------------------------------------------
					decompress_gz_file
---------------------------------------------------------------*/
bool mrpt::io::zip::decompress_gz_file(
	const std::string& file_path, std::vector<uint8_t>& buffer)
{
	CFileGZInputStream iss(file_path);
	if (!iss.fileOpenCorrectly()) return false;

	buffer.clear();

	const int bytes2read = 1 << 20;
	int act_read;
	size_t total_bytes = 0;

	buffer.reserve(iss.getTotalBytesCount());

	do
	{
		buffer.resize(1000 + bytes2read + buffer.size());
		act_read = iss.Read(&buffer[total_bytes], bytes2read);
		total_bytes += act_read;
	} while (act_read == bytes2read);

	buffer.resize(total_bytes);
	return true;
}

bool mrpt::io::zip::compress_gz_file(
	const std::string& file_path, const std::vector<uint8_t>& buffer,
	const int compress_level)
{
	CFileGZOutputStream oss;
	oss.open(file_path, compress_level);
	if (!oss.fileOpenCorrectly()) return false;

	if (!buffer.empty())
	{
		try
		{
			oss.Write(&buffer[0], buffer.size());
			return true;
		}
		catch (...)
		{
			return false;
		}
	}
	else
	{
		return true;
	}
}

bool mrpt::io::zip::compress_gz_data_block(
	const std::vector<uint8_t>& in_data, std::vector<uint8_t>& out_gz_data,
	const int compress_level)
{
	out_gz_data.clear();
	if (in_data.empty()) return true;

	const unsigned int nPipeName =
		std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::string pipe_file_name;

// Create an anonymous pipe for writing the data to:
#ifdef _WIN32
	// Windows:
	pipe_file_name =
		format("\\\\.\\pipe\\mrpt_compress_gz_data_block_%u", nPipeName);

	HANDLE hPipe = CreateNamedPipeA(
		pipe_file_name.c_str(),
		PIPE_ACCESS_DUPLEX | 0x00080000 /* FILE_FLAG_FIRST_PIPE_INSTANCE */,
		PIPE_TYPE_BYTE, PIPE_UNLIMITED_INSTANCES, in_data.size() + 1000,
		in_data.size() + 1000, 0, nullptr);
	if (hPipe == INVALID_HANDLE_VALUE)
		THROW_EXCEPTION("Error creating named pipe for gz-file compression");
#else
	// Unix:
	pipe_file_name = format("/tmp/mrpt_compress_gz_data_block_%u", nPipeName);
#endif

	bool retVal = false;
	try
	{
		// Write as gz
		{
			gzFile f = gzopen(
				pipe_file_name.c_str(), format("wb%i", compress_level).c_str());
			if (f)
			{
				retVal = (int)in_data.size() ==
						 gzwrite(f, &in_data[0], in_data.size());
				gzclose(f);
			}
			else
			{
				std::cerr << "[compress_gz_data_block] Error writing to pipe: "
						  << pipe_file_name << std::endl;
			}
		}
		// Read it all:
		if (retVal)
		{
#ifdef _WIN32
			// Read (windows)
			const size_t N = GetFileSize(hPipe, nullptr);
			if (N)
			{
				out_gz_data.resize(N);
				DWORD nRead;
				SetFilePointer(hPipe, 0, nullptr, FILE_BEGIN);
				if (N)
				{
					ReadFile(hPipe, &out_gz_data[0], N, &nRead, nullptr);
					retVal = nRead == N;
				}
			}
			else
			{
				retVal = false;
			}
#else
			// Read (Unix)
			{
				CFileInputStream iss;
				if (iss.open(pipe_file_name))
				{
					const size_t M = iss.getTotalBytesCount();
					out_gz_data.resize(M);
					if (M) retVal = M == iss.Read(&out_gz_data[0], M);
				}
			}
#endif
		}
	}
	catch (...)
	{
		retVal = false;
	}

// Close the pipe:
#ifdef _WIN32
	CloseHandle(hPipe);
#else
	remove(pipe_file_name.c_str());
#endif

	return retVal;
}

bool mrpt::io::zip::decompress_gz_data_block(
	const std::vector<uint8_t>& in_gz_data, std::vector<uint8_t>& out_data)
{
	out_data.clear();
	if (in_gz_data.empty()) return true;

	// JL: I tried to do this with pipes but had no luck... :-(
	const std::string tmp_file_name = mrpt::system::getTempFileName();
	if (!mrpt::io::vectorToBinaryFile(in_gz_data, tmp_file_name)) return false;
	bool retVal = mrpt::io::zip::decompress_gz_file(tmp_file_name, out_data);

	remove(tmp_file_name.c_str());  // Delete tmp file

	return retVal;
}
