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
