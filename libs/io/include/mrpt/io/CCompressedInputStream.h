/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/compression_options.h>

#include <memory>

namespace mrpt::io
{
/** Transparently reads from a compressed file, automatically detecting the
 * compression format from the file magic signature.
 *
 * Supports:
 * - Uncompressed files
 * - Gzip compressed files (.gz)
 * - Zstandard compressed files (.zst)
 *
 * The compression type is detected automatically when opening the file.
 *
 * \sa CFileInputStream, CCompressedOutputStream
 * \ingroup mrpt_io_grp
 */
class CCompressedInputStream : public CStream
{
 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;

  /** Compressed file size (physical size on disk) */
  uint64_t m_file_size{0};

 public:
  /** Constructor without open */
  CCompressedInputStream();

  /** Constructor and open
   * \param fileName The file to be opened in this stream
   * \exception std::exception If there's an error opening the file.
   */
  explicit CCompressedInputStream(const std::string& fileName);

  CCompressedInputStream(const CCompressedInputStream&) = delete;
  CCompressedInputStream& operator=(const CCompressedInputStream&) = delete;

  /** Destructor */
  ~CCompressedInputStream() override;

  [[nodiscard]] std::string getStreamDescription() const override;

  /** Opens the file for reading. Automatically detects compression format.
   * \param fileName The file to be opened in this stream
   * \param error_msg Optional output parameter for error message
   * \return false if there's an error opening the file, true otherwise
   */
  bool open(const std::string& fileName, mrpt::optional_ref<std::string> error_msg = std::nullopt);

  /** Closes the file */
  void close();

  /** Returns true if the file was opened without errors. */
  [[nodiscard]] bool fileOpenCorrectly() const;

  /** Returns true if the file was opened without errors. */
  [[nodiscard]] bool is_open() { return fileOpenCorrectly(); }

  /** Will be true if EOF has been already reached. */
  [[nodiscard]] bool checkEOF();

  /** Returns the path of the filename passed to open(), or empty if none. */
  [[nodiscard]] std::string filePathAtUse() const;

  /** Returns the detected compression type for the opened file. */
  [[nodiscard]] CompressionType getCompressionType() const;

  /** Method for getting the total number of **compressed** bytes in the file
   * (the physical size of the file on disk). */
  [[nodiscard]] uint64_t getTotalBytesCount() const override;

  /** Method for getting the current cursor position in the **compressed**
   * data, where 0 is the first byte and TotalBytesCount-1 the last one.
   *
   * For uncompressed files, this equals the position in uncompressed data.
   * For compressed files, this is an approximation based on the compressed
   * stream position.
   */
  [[nodiscard]] uint64_t getPosition() const override;

  /** Attempts to estimate the total uncompressed size of the file.
   * \return Estimated uncompressed size, or 0 if it cannot be determined.
   * \note For Gzip files, this reads the size from the footer if available.
   * For Zstd files with frame content size, returns that value.
   * For uncompressed files, returns the actual file size.
   * If the exact size is not available, returns an estimate based on the
   * compression ratio measured so far (compressed bytes read vs uncompressed
   * bytes served). This estimate improves as more data is read.
   * Returns 0 only if no data has been read yet and no size hint is available.
   */
  [[nodiscard]] uint64_t getUncompressedSize() const;

  /** Returns the compression ratio measured so far.
   * \return Ratio of uncompressed to compressed bytes, or 0.0 if no data read.
   * \note This is the ratio: uncompressed_bytes / compressed_bytes.
   * A ratio of 3.0 means the data compressed to 1/3 of its original size.
   */
  [[nodiscard]] double getCompressionRatio() const;

  /** Estimates the current position in the uncompressed data stream.
   * \return Estimated position in uncompressed data, or 0 if it cannot be
   * determined. \note This is an estimate and may not be accurate for all
   * compression formats.
   */
  [[nodiscard]] uint64_t getUncompressedPosition() const;

  /** This method is not implemented in this class */
  [[nodiscard]] uint64_t Seek(int64_t Offset, CStream::TSeekOrigin Origin) override;

  [[nodiscard]] size_t Read(void* Buffer, size_t Count) override;
  [[nodiscard]] size_t Write(const void* Buffer, size_t Count) override;
};

}  // namespace mrpt::io
