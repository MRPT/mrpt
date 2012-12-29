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
#ifndef  ZipCompression_H
#define  ZipCompression_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils { class CStream; }

	/** Data compression/decompression algorithms.  \ingroup mrpt_base_grp
	 */
	namespace compress
	{
		using namespace mrpt::utils;

		/** Compression using the "zip" algorithm and from/to gzip (gz) files. \ingroup mrpt_base_grp
		 */
		namespace zip
		{
			/** Compress an array of bytes into another one.
			  */
			void  BASE_IMPEXP  compress(
				void						*inData,
				size_t						inDataSize,
				std::vector<unsigned char>	&outData);

			/** Compress an array of bytes into another one.
			  */
			void  BASE_IMPEXP  compress(
				const std::vector<unsigned char>	&inData,
				std::vector<unsigned char>			&outData);

			/** Compress an array of bytes and write the result into a stream.
			  */
			void  BASE_IMPEXP  compress(
				void						*inData,
				size_t						inDataSize,
				CStream						&out);

			/** Compress an array of bytes and write the result into a stream.
			  */
			void  BASE_IMPEXP  compress(
				const std::vector<unsigned char>	&inData,
				CStream								&out);

			/** Decompress an array of bytes into another one
			  * \exception std::exception If the apriori estimated decompressed size is not enought
			  */
			void  BASE_IMPEXP  decompress(
				void						*inData,
				size_t						inDataSize,
				std::vector<unsigned char>	&outData,
				size_t						outDataEstimatedSize);

			/** Decompress an array of bytes into another one
			  * \exception std::exception If the apriori estimated decompressed size is not enought
			  */
			void  BASE_IMPEXP  decompress(
				void						*inData,
				size_t						inDataSize,
				void						*outData,
				size_t						outDataBufferSize,
				size_t						&outDataActualSize);

			/** Decompress an array of bytes into another one
			  * \exception std::exception If the apriori estimated decompressed size is not enought
			  */
			void  BASE_IMPEXP  decompress(
				CStream						&inStream,
				size_t						inDataSize,
				void						*outData,
				size_t						outDataBufferSize,
				size_t						&outDataActualSize);

			
			/** Decompress a gzip file (xxxx.gz) into a memory buffer. If the file is not a .gz file, it just read the whole file unmodified.
			  * \return true on success, false on error.
			  * \sa compress_gz_file, decompress_gz_data_block
			  */
			bool BASE_IMPEXP  decompress_gz_file(
				const std::string &file_path, 
				vector_byte & buffer);

			/** Compress a memory buffer into a gzip file (xxxx.gz).
			  *  compress_level: 0=no compression, 1=best speed, 9=maximum
			  * \return true on success, false on error.
			  * \sa decompress_gz_file, compress_gz_data_block
			  */
			bool BASE_IMPEXP  compress_gz_file(
				const std::string &file_path, 
				const vector_byte &buffer,
				const int compress_level = 9
				);

			/** Compress a memory buffer in gz-file format and return it as a block a memory.
			  *  compress_level: 0=no compression, 1=best speed, 9=maximum
			  * \return true on success, false on error.
			  * \note If in_data is empty, an empty buffer is returned in out_gz_data and no error is reported.
			  * \sa compress_gz_file, de
			  */
			bool BASE_IMPEXP  compress_gz_data_block(
				const vector_byte &in_data,
				vector_byte &out_gz_data,
				const int compress_level = 9);

			/** Decompress an array of bytes storing a gz-compressed stream of data into a memory buffer. If the input data is not recognized as a .gz file, the output data will be an exact copy of the input.
			  * \return true on success, false on error.
			  * \sa decompress_gz_file, compress_gz_data_block
			  */
			bool BASE_IMPEXP  decompress_gz_data_block(
				const vector_byte &in_gz_data,
				vector_byte &out_data);
			

		} // End of namespace
	} // End of namespace

} // End of namespace

#endif
