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

#include <mrpt/core/pimpl.h>
#include <mrpt/img/CImage.h>

#include <map>
#include <optional>
#include <string>

/*---------------------------------------------------------------
  Class
  ---------------------------------------------------------------*/
namespace mrpt::hwdrivers
{
/** \brief Decodes video files and RTSP/HTTP streams frame by frame using FFmpeg.
 *
 * Opens local video files (AVI, MPEG, ...) and IP camera streams (rtsp://,
 * http://) via openURL(). Frames are retrieved one by one with grabFrame().
 *
 * Typical use inside CCameraSensor via grabber_type=ffmpeg. Can also be used
 * directly for reading pre-recorded video files.
 *
 * \note Requires FFmpeg libraries (libavcodec, libavformat, libswscale).
 *       On Linux, install the ffmpeg development packages. On Windows,
 *       a precompiled set must be provided alongside MRPT.
 * \sa CFFMPEG_InputStream::openURL, CFFMPEG_InputStream::grabFrame
 * \ingroup mrpt_hwdrivers_grp
 */
class CFFMPEG_InputStream
{
 private:
  /** The internal ffmpeg state */
  struct Impl;
  mrpt::pimpl<Impl> m_impl;
  /** The open URL */
  std::string m_url;
  bool m_grab_as_grayscale;

 public:
  /** \brief Default constructor. No video source is open at startup. */
  CFFMPEG_InputStream();

  /** \brief Destructor. Closes the stream if open. */
  virtual ~CFFMPEG_InputStream();

  /** \brief Opens a video file or a network video stream.
   *
   * Supports local video files (e.g. "myVideo.avi") and IP camera URLs
   * (e.g. "rtsp://a.b.c.d/live.sdp"). Credentials can be embedded as
   * "rtsp://USER:PASSWORD@IP/PATH".
   *
   * \param[in] url            Path to a local file or a network stream URL.
   * \param[in] grab_as_grayscale If true, frames are converted to grayscale.
   * \param[in] verbose        If true, stream metadata is printed to cout.
   * \param[in] options        Extra FFmpeg protocol options (key-value pairs).
   * \return true on success; false on any error (details printed to cerr).
   * \sa close, grabFrame
   */
  bool openURL(
      const std::string& url,
      bool grab_as_grayscale = false,
      bool verbose = false,
      const std::map<std::string, std::string>& options = {
          {"rtsp_transport", "tcp"}
  });

  /** \brief Returns true if the video source is currently open. */
  bool isOpen() const;

  /** \brief Closes the video stream.
   *
   * Called automatically on destruction.
   * \sa openURL
   */
  void close();

  /** \brief Returns the nominal frame rate of the open video source.
   * \return Frames per second, or -1 if the stream is not open.
   */
  double getVideoFPS() const;

  /** \brief Decodes and returns the next video frame.
   *
   * For network streams this call may block until enough data is available.
   * \param[out] out_img The decoded image (8-bit RGB or grayscale).
   * \return false on any error or end of stream, true on success.
   * \deprecated Use grabFrame() instead.
   * \sa openURL, close, isOpen
   */
  [[deprecated("Use grabFrame() instead")]] bool retrieveFrame(mrpt::img::CImage& out_img);

  /** \brief Decodes the next frame and also returns its presentation timestamp.
   *
   * \param[out] out_img Decoded image.
   * \param[out] outPTS  AVFrame::pts value (ffmpeg presentation timestamp).
   * \return false on any error or end of stream, true on success.
   */
  bool retrieveFrame(mrpt::img::CImage& out_img, int64_t& outPTS);

  /** \brief Decodes and returns the next video frame by value.
   *
   * \return std::nullopt on any error or end of stream, or the image on
   * success.
   * \sa openURL, close, isOpen
   */
  [[nodiscard]] std::optional<mrpt::img::CImage> grabFrame()
  {
    mrpt::img::CImage img;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    if (!retrieveFrame(img))
#pragma GCC diagnostic pop
    {
      return std::nullopt;
    }
    return img;
  }
};
}  // namespace mrpt::hwdrivers
