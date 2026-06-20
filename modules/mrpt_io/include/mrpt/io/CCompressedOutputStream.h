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
#include <mrpt/io/open_flags.h>

#include <memory>

namespace mrpt::io
{
/** Saves data to a file with optional transparent compression.
 *
 * Supports:
 * - No compression (raw binary output)
 * - Gzip compression
 * - Zstandard compression
 *
 * The compression type and level are specified via CompressionOptions.
 *
 * \sa CFileOutputStream, CCompressedInputStream
 * \ingroup mrpt_io_grp
 */
class CCompressedOutputStream : public CStream
{
 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;

 public:
  /** Constructor: opens an output file with the given compression options.
   *
   * \param fileName The file to be opened in this stream
   * \param mode Specify whether to truncate/create the file, or to append at
   * the end if it exists. Note: append mode may not work correctly with all
   * compression formats.
   * \param options Compression type and level
   * \exception std::exception if the file cannot be opened.
   */
  explicit CCompressedOutputStream(
      const std::string& fileName,
      const OpenMode mode = OpenMode::TRUNCATE,
      const CompressionOptions& options = CompressionOptions());

  /** Constructor, without opening the file.
   * \sa open
   */
  CCompressedOutputStream();

  CCompressedOutputStream(const CCompressedOutputStream&) = delete;
  CCompressedOutputStream& operator=(const CCompressedOutputStream&) = delete;

  /** Destructor */
  ~CCompressedOutputStream() override;

  [[nodiscard]] std::string getStreamDescription() const override;

  /** Open a file for writing with the specified compression options.
   * \param fileName The file to be opened in this stream
   * \param options Compression type and level
   * \param error_msg Optional output parameter for error message
   * \param mode Specify whether to truncate or append
   * \return true on success, false on any error.
   */
  bool open(
      const std::string& fileName,
      const CompressionOptions& options = CompressionOptions(),
      mrpt::optional_ref<std::string> error_msg = std::nullopt,
      const OpenMode mode = OpenMode::TRUNCATE);

  /** Close the file */
  void close();

  /** Returns true if the file was opened without errors. */
  [[nodiscard]] bool fileOpenCorrectly() const;

  /** Returns true if the file was opened without errors. */
  [[nodiscard]] bool is_open() { return fileOpenCorrectly(); }

  /** Method for getting the current cursor position in the compressed stream,
   * where 0 is the first byte and TotalBytesCount-1 the last one.
   *
   * For compressed files, this represents the position in the compressed
   * output. For uncompressed files, it equals the number of bytes written.
   */
  [[nodiscard]] uint64_t getPosition() const override;

  /** Returns the path of the filename passed to open(), or empty if none. */
  [[nodiscard]] std::string filePathAtUse() const;

  /** Returns the compression type being used. */
  [[nodiscard]] CompressionType getCompressionType() const;

  /** This method is not implemented in this class */
  [[nodiscard]] uint64_t Seek(int64_t Offset, CStream::TSeekOrigin Origin) override;

  /** This method is not implemented in this class */
  [[nodiscard]] uint64_t getTotalBytesCount() const override;

  [[nodiscard]] size_t Read(void* Buffer, size_t Count) override;
  [[nodiscard]] size_t Write(const void* Buffer, size_t Count) override;
};

static_assert(
    !std::is_copy_constructible_v<CCompressedOutputStream> &&
        !std::is_copy_assignable_v<CCompressedOutputStream>,
    "Copy Check");

}  // namespace mrpt::io
