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

#include <mrpt/img/CImage.h>
#include <mrpt/img/TPixelCoord.h>

#include <memory>
#include <string>

namespace mrpt::img
{
/** Selects the encoding used by CVideoFileWriter.
 *
 * - `UncompressedRGB`: Raw BGR24 frames stored in an AVI (BI_RGB). Large
 *   files, zero quality loss, no additional dependencies beyond file I/O.
 *
 * - `MJPEG`: Each frame is an independent JPEG, stored in an AVI as an
 *   MJPG stream. Good compression, still widely compatible, and requires
 *   only the STB JPEG encoder already present in MRPT (no external libs).
 *
 * \ingroup mrpt_img_grp
 */
enum class VideoCodec : uint8_t
{
  UncompressedRGB = 0,  //!< Raw BGR24 in AVI/RIFF container
  MJPEG = 1,            //!< Motion-JPEG in AVI/RIFF container (uses STB)
};

/** Writes a sequence of CImage frames to a video file.
 *
 * Supports dependency-free AVI output via two codecs (see VideoCodec).
 * The file is opened with open() and finalised/closed with close() or at
 * destruction.
 *
 * Example:
 * \code
 *   mrpt::img::CVideoFileWriter vid;
 *   vid.open("out.avi", 30.0, {640, 480});          // MJPEG by default
 *   // -- or --
 *   vid.open("out.avi", 30.0, {640, 480},
 *            mrpt::img::VideoCodec::UncompressedRGB);
 *
 *   mrpt::img::CImage img(640, 480, mrpt::img::CH_RGB);
 *   vid << img;          // throws on error
 *   vid.writeImage(img); // returns false on error instead
 *   vid.close();
 * \endcode
 *
 * \note Replaces the old mrpt::vision::CVideoFileWriter which depended on
 *       OpenCV's CvVideoWriter. The new implementation writes RIFF/AVI
 *       directly and uses STB for JPEG encoding (MJPEG path), so there are
 *       no external library dependencies beyond what mrpt_img already pulls
 *       in.
 *
 * \note Both codecs produce standard AVI files (RIFF, OpenDML-compatible
 *       index) readable by ffmpeg, VLC, and any DirectShow/GStreamer stack.
 *
 * \ingroup mrpt_img_grp
 */
class CVideoFileWriter
{
 public:
  /** Default constructor. No file is opened yet. */
  CVideoFileWriter();

  /** Destructor - calls close() if the file is still open. */
  ~CVideoFileWriter();

  // Non-copyable (owns a raw FILE* and mutable index state)
  CVideoFileWriter(const CVideoFileWriter&) = delete;
  CVideoFileWriter& operator=(const CVideoFileWriter&) = delete;

  // Movable
  CVideoFileWriter(CVideoFileWriter&&) noexcept;
  CVideoFileWriter& operator=(CVideoFileWriter&&) noexcept;

  /** Open a file and write the AVI headers.
   *
   * \param out_file   Path to the output .avi file (created/truncated).
   * \param fps        Frames per second (e.g. 25.0, 29.97, 30.0).
   * \param frameSize  Width × height of every frame that will be written.
   *                   All subsequent images *must* match this size exactly.
   * \param codec      Encoding to use (default: MJPEG).
   * \param jpeg_quality  Quality for MJPEG frames, 1-100 (ignored for
   *                      UncompressedRGB). Higher = better quality / larger
   *                      file. Default: 90.
   *
   * \return true on success, false on any I/O error.
   *
   * \note The old `isColor` parameter has been removed. Color mode is
   *       derived automatically from each frame's CImage::channels().
   *       Grayscale images are written as 8-bit luminance planes; color
   *       images are written as BGR24 (or MJPEG BGR). Mixed sequences are
   *       not supported - use a consistent channel count throughout.
   */
  [[nodiscard]] bool open(
      const std::string& out_file,
      double fps,
      const TImageSize& frameSize,
      VideoCodec codec = VideoCodec::MJPEG,
      int jpeg_quality = 90);

  /** Flush and finalise the AVI file (writes the idx1 index and fixes up
   * all size fields in the RIFF/LIST headers). Safe to call more than once.
   */
  void close();

  /** Returns true if a file is currently open. */
  [[nodiscard]] bool isOpen() const noexcept;

  /** Returns the codec chosen at open(). Undefined if !isOpen(). */
  [[nodiscard]] VideoCodec codec() const noexcept;

  /** Returns the frame size chosen at open(). Undefined if !isOpen(). */
  [[nodiscard]] TImageSize frameSize() const noexcept;

  /** Returns the number of frames written so far. */
  [[nodiscard]] uint32_t frameCount() const noexcept;

  /** Write one frame to the video.
   * \exception std::runtime_error on any error (image size mismatch, I/O
   *            failure, or writing to a closed writer).
   */
  const CVideoFileWriter& operator<<(const mrpt::img::CImage& img);

  /** Write one frame to the video.
   * \return false on any error (prefer this in performance-critical loops
   *         where exception overhead matters).
   */
  [[nodiscard]] bool writeImage(const mrpt::img::CImage& img);

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace mrpt::img