/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>

namespace mrpt
{
namespace io
{
class CStream;

/** Compression using the "zip" algorithm and from/to gzip (gz) files.
 * \ingroup mrpt_io_grp */
namespace zip
{
/** Compress an array of bytes into another one. */
void compress(
	void* inData, size_t inDataSize, std::vector<unsigned char>& outData);

/** Compress an array of bytes into another one. */
void compress(
	const std::vector<unsigned char>& inData,
	std::vector<unsigned char>& outData);

/** Compress an array of bytes and write the result into a stream. */
void compress(void* inData, size_t inDataSize, mrpt::io::CStream& out);

/** Compress an array of bytes and write the result into a stream. */
void compress(const std::vector<unsigned char>& inData, mrpt::io::CStream& out);

/** Decompress an array of bytes into another one
 * \exception std::exception If the apriori estimated decompressed size is not
 * enough */
void decompress(
	void* inData, size_t inDataSize, std::vector<unsigned char>& outData,
	size_t outDataEstimatedSize);

/** Decompress an array of bytes into another one
 * \exception std::exception If the apriori estimated decompressed size is not
 * enough
 */
void decompress(
	void* inData, size_t inDataSize, void* outData, size_t outDataBufferSize,
	size_t& outDataActualSize);

/** Decompress an array of bytes into another one
 * \exception std::exception If the apriori estimated decompressed size is not
 * enough
 */
void decompress(
	mrpt::io::CStream& inStream, size_t inDataSize, void* outData,
	size_t outDataBufferSize, size_t& outDataActualSize);

/** Decompress a gzip file (xxxx.gz) into a memory buffer. If the file is not a
 * .gz file, it just read the whole file unmodified.
 * \return true on success, false on error.
 * \sa compress_gz_file, decompress_gz_data_block
 */
bool decompress_gz_file(
	const std::string& file_path, std::vector<uint8_t>& buffer);

/** Compress a memory buffer into a gzip file (xxxx.gz).
 *  compress_level: 0=no compression, 1=best speed, 9=maximum
 * \return true on success, false on error.
 * \sa decompress_gz_file, compress_gz_data_block
 */
bool compress_gz_file(
	const std::string& file_path, const std::vector<uint8_t>& buffer,
	const int compress_level = 9);

/** Compress a memory buffer in gz-file format and return it as a block a
 * memory.
 *  compress_level: 0=no compression, 1=best speed, 9=maximum
 * \return true on success, false on error.
 * \note If in_data is empty, an empty buffer is returned in out_gz_data and no
 * error is reported.
 * \sa compress_gz_file, de
 */
bool compress_gz_data_block(
	const std::vector<uint8_t>& in_data, std::vector<uint8_t>& out_gz_data,
	const int compress_level = 9);

/** Decompress an array of bytes storing a gz-compressed stream of data into a
 * memory buffer. If the input data is not recognized as a .gz file, the output
 * data will be an exact copy of the input.
 * \return true on success, false on error.
 * \sa decompress_gz_file, compress_gz_data_block
 */
bool decompress_gz_data_block(
	const std::vector<uint8_t>& in_gz_data, std::vector<uint8_t>& out_data);

}  // namespace zip
}  // namespace io
}  // namespace mrpt
