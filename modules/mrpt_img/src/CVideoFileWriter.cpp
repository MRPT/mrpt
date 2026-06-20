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

// ============================================================
// AVI / RIFF file format overview (no-dependency implementation)
// ============================================================
//
// The RIFF/AVI layout written by this class:
//
//  RIFF('AVI ')                          <- top-level chunk
//    LIST('hdrl')                        <- header list
//      'avih'  (AviMainHeader)           <- file-level metadata
//      LIST('strl')                      <- one stream list (video only)
//        'strh'  (AviStreamHeader)       <- stream type / timing
//        'strf'  (BitmapInfoHeader)      <- pixel format
//          [RGBQUAD[256]]                <- grayscale palette (gray streams only)
//    LIST('movi')                        <- frame data
//      '00dc' / '00db'  (frame bytes)   <- one chunk per frame
//        [dc = compressed (MJPEG), db = uncompressed DIB]
//  'idx1'  (AviIndexEntry[])             <- legacy OpenDML index
//
// All multi-byte integers are little-endian (x86 native).
// Size fields that depend on frame data are patched with fseek/fwrite
// during close().
//
// Channel-order notes
// -------------------
// CImage::getChannelsOrder() returns "GRAY", "RGB", or "RGBA" - i.e. the
// in-memory channel order is always R-first for color images.
//
// AVI uncompressed DIB (BI_RGB): expects BGR, bottom-up rows, DWORD-aligned.
//   Color → copy rows in reverse order AND swap channel bytes 0↔2 (R↔B).
//   Gray  → 8-bit bottom-up rows, DWORD-aligned, no channel swap.
//
// AVI MJPEG: STB's stbi_write_jpg_to_func interprets comp=3 data as RGB,
//            comp=1 as gray, comp=4 as RGBA (drops alpha for JPEG output).
//            Because CImage is also RGB-first and top-down, MJPEG encoding
//            can be done with a direct (zero-copy) pointer into CImage's
//            buffer whenever the image rows are contiguous in memory.
//
// Header deferral
// ---------------
// The BitmapInfoHeader inside 'strf' (and the optional grayscale palette)
// depends on whether the stream is gray or color.  Because open() does not
// receive that information, headers are written lazily on the first call to
// appendFrame(), once the channel layout is known from the actual image.
//
// References:
//   https://learn.microsoft.com/en-us/windows/win32/directshow/avi-riff-file-reference
//   https://www.fourcc.org/rgb.php
// ============================================================

#include <mrpt/core/config.h>  // MRPT_IS_BIG_ENDIAN
#if MRPT_IS_BIG_ENDIAN
#include <mrpt/core/reverse_bytes.h>
#endif

#include <mrpt/img/CVideoFileWriter.h>

// STB - JPEG encoding for MJPEG path.
// The implementation unit defines the implementation macro exactly once.
// Adjust the include path to wherever stb_image_write.h lives in your tree.
// #define STB_IMAGE_WRITE_IMPLEMENTATION // Defined only in CImage.cpp
#include <stb/stb_image_write.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace
{
// ── Platform / byte-order helpers ──────────────────────────────────────────

// All AVI fields are little-endian. On LE hosts (all common targets) these
// are no-ops; kept for explicitness and future BE-port hygiene.
#if MRPT_IS_BIG_ENDIAN
inline uint16_t le16(uint16_t v) { return mrpt::reverseBytes(v); }
inline uint32_t le32(uint32_t v) { return v; }
#else
inline uint16_t le16(uint16_t v) { return v; }
inline uint32_t le32(uint32_t v) { return v; }
#endif

// ── RIFF four-character codes ──────────────────────────────────────────────

constexpr uint32_t fourCC(const char s[5])
{
  return static_cast<uint32_t>(s[0]) | (static_cast<uint32_t>(s[1]) << 8) |
         (static_cast<uint32_t>(s[2]) << 16) | (static_cast<uint32_t>(s[3]) << 24);
}

// ── Packed AVI structs (RIFF wire layout) ─────────────────────────────────

#pragma pack(push, 1)

/** Generic RIFF chunk header: fourCC + payload size (excl. this 8-byte hdr). */
struct RiffChunkHeader
{
  uint32_t fcc{0};
  uint32_t size{0};
};
static_assert(sizeof(RiffChunkHeader) == 8);

/** MainAVIHeader ('avih') */
struct AviMainHeader
{
  uint32_t dwMicroSecPerFrame{0};     //!< Frame duration in µs = 1e6/fps
  uint32_t dwMaxBytesPerSec{0};       //!< Upper bound on stream bitrate; patched at close()
  uint32_t dwPaddingGranularity{0};   //!< Alignment for data, typically 0
  uint32_t dwFlags{0};                //!< AVIF_* flags
  uint32_t dwTotalFrames{0};          //!< Patched at close()
  uint32_t dwInitialFrames{0};        //!< 0 for non-interleaved
  uint32_t dwStreams{1};              //!< Number of streams (1 = video only)
  uint32_t dwSuggestedBufferSize{0};  //!< Largest expected frame; patched at close()
  uint32_t dwWidth{0};
  uint32_t dwHeight{0};
  uint32_t dwReserved[4]{0, 0, 0, 0};
};
static_assert(sizeof(AviMainHeader) == 56);

/** AVIStreamHeader ('strh') */
struct AviStreamHeader
{
  uint32_t fccType{0};     //!< 'vids'
  uint32_t fccHandler{0};  //!< 'MJPG', or 0 for uncompressed DIB
  uint32_t dwFlags{0};
  uint16_t wPriority{0};
  uint16_t wLanguage{0};
  uint32_t dwInitialFrames{0};
  uint32_t dwScale{0};  //!< Together with dwRate: fps = dwRate/dwScale
  uint32_t dwRate{0};
  uint32_t dwStart{0};
  uint32_t dwLength{0};               //!< Total frames; patched at close()
  uint32_t dwSuggestedBufferSize{0};  //!< Patched at close()
  uint32_t dwQuality{0xFFFFFFFF};     //!< Default quality
  uint32_t dwSampleSize{0};
  struct
  {
    int16_t left, top, right, bottom;
  } rcFrame{0, 0, 0, 0};
};
static_assert(sizeof(AviStreamHeader) == 56);

/** BITMAPINFOHEADER - used inside 'strf' for video streams. */
struct BitmapInfoHeader
{
  uint32_t biSize{40};
  int32_t biWidth{0};
  int32_t biHeight{0};  //!< Positive = bottom-up (AVI/DIB convention)
  uint16_t biPlanes{1};
  uint16_t biBitCount{0};     //!< 24 for BGR24, 8 for grayscale, 0/24 for MJPEG
  uint32_t biCompression{0};  //!< BI_RGB(0) or fourCC("MJPG")
  uint32_t biSizeImage{0};
  int32_t biXPelsPerMeter{0};
  int32_t biYPelsPerMeter{0};
  uint32_t biClrUsed{0};  //!< 256 for 8-bit palettised, 0 otherwise
  uint32_t biClrImportant{0};
};
static_assert(sizeof(BitmapInfoHeader) == 40);

/** RGBQUAD - one entry in a DIB colour table. */
struct RgbQuad
{
  uint8_t rgbBlue{0};
  uint8_t rgbGreen{0};
  uint8_t rgbRed{0};
  uint8_t rgbReserved{0};
};
static_assert(sizeof(RgbQuad) == 4);

/** One entry in the legacy 'idx1' index. */
struct AviIndexEntry
{
  uint32_t ckid{0};           //!< Chunk fourCC ('00dc' or '00db')
  uint32_t dwFlags{0x10};     //!< AVIIF_KEYFRAME = 0x10
  uint32_t dwChunkOffset{0};  //!< Offset from start of 'movi' data (after its 8-byte LIST header)
  uint32_t dwChunkSize{0};
};
static_assert(sizeof(AviIndexEntry) == 16);

#pragma pack(pop)

// ── Low-level RIFF write helpers ───────────────────────────────────────────

/** Write a fourCC + size header; returns the file position of the size field. */
long writeChunkHeader(FILE* fp, uint32_t fcc, uint32_t size = 0)
{
  RiffChunkHeader hdr{le32(fcc), le32(size)};
  std::fwrite(&hdr, sizeof(hdr), 1, fp);
  return std::ftell(fp) - static_cast<long>(sizeof(uint32_t));
}

/** Write a RIFF/LIST opener (fourCC + size + type); returns the offset of size field. */
long writeListHeader(FILE* fp, uint32_t listFcc, uint32_t typeFcc, uint32_t size = 0)
{
  RiffChunkHeader hdr{le32(listFcc), le32(size)};
  std::fwrite(&hdr, sizeof(hdr), 1, fp);
  long sizePos = std::ftell(fp) - static_cast<long>(sizeof(uint32_t));
  uint32_t t = le32(typeFcc);
  std::fwrite(&t, sizeof(t), 1, fp);
  return sizePos;
}

/** Seek back and overwrite a previously-written uint32_t size field. */
void patchSize(FILE* fp, long sizeFieldPos, uint32_t value)
{
  const long saved = std::ftell(fp);
  std::fseek(fp, sizeFieldPos, SEEK_SET);
  const uint32_t v = le32(value);
  std::fwrite(&v, sizeof(v), 1, fp);
  std::fseek(fp, saved, SEEK_SET);
}

// Write a 256-entry linear grayscale RGBQUAD palette (shared by both codecs).
void writeGrayscalePalette(FILE* fp)
{
  for (int i = 0; i < 256; ++i)
  {
    RgbQuad q{static_cast<uint8_t>(i), static_cast<uint8_t>(i), static_cast<uint8_t>(i), 0};
    std::fwrite(&q, sizeof(q), 1, fp);
  }
}

// ── STB JPEG write callback ────────────────────────────────────────────────

/** stb_image_write callback that appends bytes to a std::vector<uint8_t>. */
void stbJpegCallback(void* ctx, void* data, int size)
{
  auto* buf = static_cast<std::vector<uint8_t>*>(ctx);
  const auto* src = static_cast<const uint8_t*>(data);
  buf->insert(buf->end(), src, src + size);
}

}  // namespace
// ── Impl ───────────────────────────────────────────────────────────────────

namespace mrpt::img
{

struct CVideoFileWriter::Impl
{
  FILE* fp{nullptr};
  VideoCodec codec{VideoCodec::MJPEG};
  int width{0};
  int height{0};
  double fps{0.0};
  int jpegQuality{90};
  uint32_t frameCount{0};
  uint32_t maxFrameBytes{0};  //!< Largest frame written (for dwSuggestedBufferSize)

  /** Set from the first frame; all subsequent frames must match. */
  bool isGrayscale{false};

  /** Headers are deferred to the first appendFrame() call so that we
   *  know the actual channel layout before writing the BitmapInfoHeader. */
  bool headersWritten{false};

  // File positions of fields that must be patched at close():
  long pos_riff_size{0};
  long pos_hdrl_size{0};
  long pos_avih_totalframes{0};
  long pos_avih_maxbytespersec{0};
  long pos_avih_suggestedbuf{0};
  long pos_strh_length{0};
  long pos_strh_suggestedbuf{0};
  long pos_movi_size{0};
  long pos_movi_data_start{0};  //!< Byte offset just after the 'movi' LIST type fourCC

  uint32_t frameFcc{0};  //!< '00dc' (compressed) or '00db' (uncompressed)

  std::vector<AviIndexEntry> index;

  // ── helpers ──

  bool writeHeaders();
  bool appendFrame(const mrpt::img::CImage& img);
  bool appendFrameUncompressed(const mrpt::img::CImage& img);
  bool appendFrameMJPEG(const mrpt::img::CImage& img);
  bool finalizeFile();

  Impl() = default;
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  Impl(Impl&&) = default;
  Impl& operator=(Impl&&) = default;

  ~Impl()
  {
    if (fp != nullptr)
    {
      finalizeFile();
    }
  }
};

// ── writeHeaders ──────────────────────────────────────────────────────────
//
// Called lazily on the first appendFrame() after isGrayscale has been set.

bool CVideoFileWriter::Impl::writeHeaders()
{
  const bool isMJPEG = (codec == VideoCodec::MJPEG);

  // -- RIFF 'AVI ' -----------------------------------------------------------
  pos_riff_size = writeListHeader(fp, fourCC("RIFF"), fourCC("AVI "), 0);

  // -- LIST 'hdrl' -----------------------------------------------------------
  pos_hdrl_size = writeListHeader(fp, fourCC("LIST"), fourCC("hdrl"), 0);

  // -- 'avih' ----------------------------------------------------------------
  {
    AviMainHeader avih{};
    avih.dwMicroSecPerFrame = le32(static_cast<uint32_t>(std::lround(1'000'000.0 / fps)));
    avih.dwMaxBytesPerSec = le32(0);  // patched at close()
    avih.dwFlags = le32(0x00000110);  // AVIF_HASINDEX | AVIF_ISINTERLEAVED
    avih.dwTotalFrames = le32(0);     // patched at close()
    avih.dwStreams = le32(1);
    avih.dwSuggestedBufferSize = le32(0);  // patched at close()
    avih.dwWidth = le32(static_cast<uint32_t>(width));
    avih.dwHeight = le32(static_cast<uint32_t>(height));

    writeChunkHeader(fp, fourCC("avih"), sizeof(AviMainHeader));
    const long avihStart = std::ftell(fp);
    pos_avih_totalframes = avihStart + static_cast<long>(offsetof(AviMainHeader, dwTotalFrames));
    pos_avih_maxbytespersec =
        avihStart + static_cast<long>(offsetof(AviMainHeader, dwMaxBytesPerSec));
    pos_avih_suggestedbuf =
        avihStart + static_cast<long>(offsetof(AviMainHeader, dwSuggestedBufferSize));
    std::fwrite(&avih, sizeof(avih), 1, fp);
  }

  // -- LIST 'strl' -----------------------------------------------------------
  const long pos_strl_size = writeListHeader(fp, fourCC("LIST"), fourCC("strl"), 0);

  // -- 'strh' ----------------------------------------------------------------
  {
    AviStreamHeader strh{};
    strh.fccType = le32(fourCC("vids"));
    strh.fccHandler = le32(isMJPEG ? fourCC("MJPG") : 0u);  // 0 = BI_RGB for uncompressed

    // Express fps as exact rational with denominator 1000 (handles non-integer rates).
    const uint32_t scale = 1000;
    const auto rate = static_cast<uint32_t>(std::lround(fps * scale));
    strh.dwScale = le32(scale);
    strh.dwRate = le32(rate);
    strh.dwLength = le32(0);               // patched at close()
    strh.dwSuggestedBufferSize = le32(0);  // patched at close()
    strh.rcFrame.right = static_cast<int16_t>(width);
    strh.rcFrame.bottom = static_cast<int16_t>(height);

    writeChunkHeader(fp, fourCC("strh"), sizeof(AviStreamHeader));
    const long strhStart = std::ftell(fp);
    pos_strh_length = strhStart + static_cast<long>(offsetof(AviStreamHeader, dwLength));
    pos_strh_suggestedbuf =
        strhStart + static_cast<long>(offsetof(AviStreamHeader, dwSuggestedBufferSize));
    std::fwrite(&strh, sizeof(strh), 1, fp);
  }

  // -- 'strf' (BitmapInfoHeader [+ palette]) ---------------------------------
  //
  // Gray streams (both codecs) require a 256-entry RGBQUAD linear palette
  // appended immediately after BitmapInfoHeader; the strf chunk size must
  // include it.  The biClrUsed / biClrImportant fields are set to 256.
  {
    BitmapInfoHeader bmih{};
    bmih.biWidth = static_cast<int32_t>(le32(width));
    bmih.biHeight = static_cast<int32_t>(le32(height));  // positive = bottom-up (AVI convention)
    bmih.biPlanes = le16(1);

    if (isMJPEG)
    {
      // The BITMAPINFOHEADER describes the *decoded* (output) frame format.
      bmih.biBitCount = le16(isGrayscale ? 8u : 24u);
      bmih.biCompression = le32(fourCC("MJPG"));
      bmih.biSizeImage = le32(0);  // variable per JPEG frame
      if (isGrayscale)
      {
        bmih.biClrUsed = le32(256);
        bmih.biClrImportant = le32(256);
        const uint32_t strfSz =
            sizeof(BitmapInfoHeader) + 256u * static_cast<uint32_t>(sizeof(RgbQuad));
        writeChunkHeader(fp, fourCC("strf"), strfSz);
        std::fwrite(&bmih, sizeof(bmih), 1, fp);
        writeGrayscalePalette(fp);
      }
      else
      {
        writeChunkHeader(fp, fourCC("strf"), sizeof(BitmapInfoHeader));
        std::fwrite(&bmih, sizeof(bmih), 1, fp);
      }
    }
    else  // UncompressedRGB
    {
      if (isGrayscale)
      {
        // 8-bit palettised DIB.  DIB rows are DWORD-aligned: (width+3)&~3.
        const int rowStride = (width + 3) & ~3;
        bmih.biBitCount = le16(8);
        bmih.biCompression = le32(0);  // BI_RGB
        bmih.biSizeImage = le32(static_cast<uint32_t>(rowStride * height));
        bmih.biClrUsed = le32(256);
        bmih.biClrImportant = le32(256);
        const uint32_t strfSz =
            sizeof(BitmapInfoHeader) + 256u * static_cast<uint32_t>(sizeof(RgbQuad));
        writeChunkHeader(fp, fourCC("strf"), strfSz);
        std::fwrite(&bmih, sizeof(bmih), 1, fp);
        writeGrayscalePalette(fp);
      }
      else
      {
        // BGR24, bottom-up, BI_RGB.
        bmih.biBitCount = le16(24);
        bmih.biCompression = le32(0);  // BI_RGB
        bmih.biSizeImage = le32(static_cast<uint32_t>(((width * 3 + 3) & ~3) * height));
        writeChunkHeader(fp, fourCC("strf"), sizeof(BitmapInfoHeader));
        std::fwrite(&bmih, sizeof(bmih), 1, fp);
      }
    }
  }

  // Patch 'strl' and 'hdrl' LIST sizes now that we know their total extent.
  {
    const long endPos = std::ftell(fp);
    // LIST payload = everything after the 8-byte chunk header + 4-byte type fourCC.
    const auto listPayload = [&](long sizePos) -> uint32_t
    {
      return static_cast<uint32_t>(
          endPos - (sizePos + static_cast<long>(sizeof(uint32_t))  // size field itself
                    + static_cast<long>(sizeof(uint32_t))));       // type fourCC
    };
    patchSize(fp, pos_strl_size, listPayload(pos_strl_size));
    patchSize(fp, pos_hdrl_size, listPayload(pos_hdrl_size));
  }

  // -- LIST 'movi' -----------------------------------------------------------
  pos_movi_size = writeListHeader(fp, fourCC("LIST"), fourCC("movi"), 0);
  pos_movi_data_start = std::ftell(fp);  // just after the 'movi' type fourCC

  // Per-frame chunk fourCC: compressed = '00dc', uncompressed = '00db'.
  frameFcc = isMJPEG ? fourCC("00dc") : fourCC("00db");

  return (std::ferror(fp) == 0);
}

// ── appendFrame ───────────────────────────────────────────────────────────

bool CVideoFileWriter::Impl::appendFrame(const mrpt::img::CImage& img)
{
  // Lazy header write: deferred here so we know the channel layout.
  if (!headersWritten)
  {
    isGrayscale = !img.isColor();
    headersWritten = true;
    if (!writeHeaders())
    {
      return false;
    }
  }

  // Validate frame dimensions.
  if (img.getWidth() != width || img.getHeight() != height)
  {
    return false;
  }

  // Validate channel consistency (cannot mix gray and color frames).
  if ((!img.isColor()) != isGrayscale)
  {
    return false;
  }

  return (codec == VideoCodec::UncompressedRGB) ? appendFrameUncompressed(img)
                                                : appendFrameMJPEG(img);
}

// ── appendFrameUncompressed ───────────────────────────────────────────────
//
// Writes a raw DIB ('00db') chunk.
//
// Grayscale: 8-bit luminance, bottom-up, rows DWORD-aligned.
//
// Color (RGB / RGBA source → BGR24 output):
//   • AVI DIB expects BGR byte order; CImage is RGB-first.
//   • Rows are reversed (bottom-up) and bytes 0↔2 of each pixel are swapped.
//   • Alpha channel (if present) is simply dropped.
//   • Output rows are DWORD-aligned (padding appended as needed).

bool CVideoFileWriter::Impl::appendFrameUncompressed(const mrpt::img::CImage& img)
{
  if (isGrayscale)
  {
    // DWORD-aligned row stride for 8-bit DIB.
    const int rowStride = (width + 3) & ~3;
    const int padBytes = rowStride - width;
    const uint32_t chunkSize = static_cast<uint32_t>(rowStride * height);

    const long chunkOffset = std::ftell(fp) - pos_movi_data_start;
    writeChunkHeader(fp, frameFcc, chunkSize);

    static const uint8_t kZero[4] = {0, 0, 0, 0};
    for (int row = height - 1; row >= 0; --row)  // bottom-up
    {
      const auto* src = img.ptrLine<uint8_t>(row);
      std::fwrite(src, 1, static_cast<size_t>(width), fp);
      if (padBytes > 0)
      {
        std::fwrite(kZero, 1, static_cast<size_t>(padBytes), fp);
      }
    }
    // RIFF chunk payloads must be WORD-aligned; pad if needed.
    if ((chunkSize & 1u) != 0)
    {
      const uint8_t pad = 0;
      std::fwrite(&pad, 1, 1, fp);
    }

    index.push_back(
        {le32(frameFcc), le32(0x10), le32(static_cast<uint32_t>(chunkOffset)), le32(chunkSize)});
    maxFrameBytes = std::max(maxFrameBytes, chunkSize);
  }
  else
  {
    // BGR24 output, bottom-up, DWORD-aligned rows.
    const int bgrRowStride = ((width * 3) + 3) & ~3;
    const int padBytes = bgrRowStride - width * 3;
    const uint32_t chunkSize = static_cast<uint32_t>(bgrRowStride * height);

    const long chunkOffset = std::ftell(fp) - pos_movi_data_start;
    writeChunkHeader(fp, frameFcc, chunkSize);

    // srcChannels is 3 (RGB) or 4 (RGBA); the channel stride tells us how
    // many bytes to advance per source pixel.
    const int srcChannels = static_cast<int>(img.channels());
    static const uint8_t kZero[4] = {0, 0, 0, 0};

    for (int row = height - 1; row >= 0; --row)  // bottom-up
    {
      const auto* src = img.ptrLine<uint8_t>(row);
      for (int col = 0; col < width; ++col)
      {
        // CImage: R=src[0], G=src[1], B=src[2]  (RGBA: A=src[3], ignored)
        // AVI DIB BGR24: B at byte 0, G at byte 1, R at byte 2
        const uint8_t bgr[3] = {src[2], src[1], src[0]};
        std::fwrite(bgr, 1, 3, fp);
        src += srcChannels;
      }
      if (padBytes > 0)
      {
        std::fwrite(kZero, 1, static_cast<size_t>(padBytes), fp);
      }
    }
    if ((chunkSize & 1u) != 0)
    {
      const uint8_t pad = 0;
      std::fwrite(&pad, 1, 1, fp);
    }

    index.push_back(
        {le32(frameFcc), le32(0x10), le32(static_cast<uint32_t>(chunkOffset)), le32(chunkSize)});
    maxFrameBytes = std::max(maxFrameBytes, chunkSize);
  }

  ++frameCount;
  return (std::ferror(fp) == 0);
}

// ── appendFrameMJPEG ─────────────────────────────────────────────────────
//
// Encodes one frame as a JPEG with STB and stores it as a '00dc' chunk.
//
// Zero-copy fast path
// -------------------
// STB stbi_write_jpg_to_func(cb, ctx, w, h, comp, data, quality) expects:
//   comp=1  gray  (matches CImage CH_GRAY)
//   comp=3  RGB   (matches CImage CH_RGB  — same R-first order)
//   comp=4  RGBA  (STB silently drops alpha when writing JPEG)
//
// Because CImage is top-down and the channel order already matches STB's
// conventions, we can pass img.ptrLine<uint8_t>(0) directly to STB
// whenever the image rows are contiguous in memory:
//   contiguous ⟺ getRowStride() == width * channels
//
// No R↔B swap is ever needed for the MJPEG path.
//
// Copy / strip path
// -----------------
// If rows carry extra padding, we compact into a temporary contiguous buffer.
// RGBA is additionally stripped to RGB to avoid writing 4-channel JPEGs
// (JPEG has no alpha; the extra channel wastes STB's effort and confuses some
// decoders that inspect SOF component counts).

bool CVideoFileWriter::Impl::appendFrameMJPEG(const mrpt::img::CImage& img)
{
  const int srcChannels = static_cast<int>(img.channels());  // 1, 3, or 4
  const size_t srcStride = img.getRowStride();
  const bool isContiguous = (srcStride == (static_cast<size_t>(width) * srcChannels));

  // For RGBA we strip the alpha channel → write a 3-component JPEG.
  const bool needsAlphaStrip = (srcChannels == 4);
  const int stbComp = needsAlphaStrip ? 3 : srcChannels;

  std::vector<uint8_t> jpegBuf;
  jpegBuf.reserve(static_cast<size_t>(width * height * stbComp / 4));

  if (isContiguous && !needsAlphaStrip)
  {
    // ── Zero-copy path ────────────────────────────────────────────────────
    // The image buffer is one contiguous block starting at row 0.
    // CImage top-down == STB top-down; CImage RGB == STB comp=3 RGB.
    // No copy, no channel swap needed.
    const auto* data = img.ptrLine<uint8_t>(0);
    stbi_write_jpg_to_func(
        stbJpegCallback, &jpegBuf, width, height, srcChannels, data, jpegQuality);
  }
  else
  {
    // ── Copy / strip path ─────────────────────────────────────────────────
    // Either rows have extra stride padding, or we need to drop alpha.
    std::vector<uint8_t> contiguous(static_cast<size_t>(width * stbComp * height));
    uint8_t* dst = contiguous.data();

    if (!needsAlphaStrip)
    {
      // Non-contiguous rows only: compact each row (no channel reordering).
      const auto rowBytes = static_cast<size_t>(width * srcChannels);
      for (int row = 0; row < height; ++row)
      {
        std::memcpy(dst, img.ptrLine<uint8_t>(row), rowBytes);
        dst += rowBytes;
      }
    }
    else
    {
      // RGBA → RGB: copy R, G, B and skip A for every pixel.
      for (int row = 0; row < height; ++row)
      {
        const auto* src = img.ptrLine<uint8_t>(row);
        for (int col = 0; col < width; ++col)
        {
          *dst++ = src[0];  // R
          *dst++ = src[1];  // G
          *dst++ = src[2];  // B
          src += 4;         // skip A
        }
      }
    }

    stbi_write_jpg_to_func(
        stbJpegCallback, &jpegBuf, width, height, stbComp, contiguous.data(), jpegQuality);
  }

  if (jpegBuf.empty())
  {
    return false;
  }

  const auto chunkSize = static_cast<uint32_t>(jpegBuf.size());
  const uint32_t paddedSize = (chunkSize + 1u) & ~1u;

  const long chunkOffset = std::ftell(fp) - pos_movi_data_start;
  writeChunkHeader(fp, frameFcc, chunkSize);
  std::fwrite(jpegBuf.data(), 1, chunkSize, fp);
  if (paddedSize > chunkSize)
  {
    const uint8_t pad = 0;
    std::fwrite(&pad, 1, 1, fp);
  }

  index.push_back(
      {le32(frameFcc), le32(0x10), le32(static_cast<uint32_t>(chunkOffset)), le32(chunkSize)});
  maxFrameBytes = std::max(maxFrameBytes, chunkSize);

  ++frameCount;
  return (std::ferror(fp) == 0);
}

// ── finalizeFile ──────────────────────────────────────────────────────────

bool CVideoFileWriter::Impl::finalizeFile()
{
  if (fp == nullptr)
  {
    return true;
  }

  // Patch movi LIST size.
  {
    const long moviEnd = std::ftell(fp);
    const auto moviPayload = static_cast<uint32_t>(
        moviEnd - (pos_movi_size + static_cast<long>(sizeof(uint32_t)) +
                   static_cast<long>(sizeof(uint32_t))));
    patchSize(fp, pos_movi_size, moviPayload);
  }

  // Write idx1 index.
  if (!index.empty())
  {
    const auto idxBytes = static_cast<uint32_t>(index.size() * sizeof(AviIndexEntry));
    writeChunkHeader(fp, fourCC("idx1"), idxBytes);
    std::fwrite(index.data(), sizeof(AviIndexEntry), index.size(), fp);
  }

  // Patch top-level RIFF size (everything after the leading 8-byte header).
  {
    const long fileEnd = std::ftell(fp);
    patchSize(fp, pos_riff_size, static_cast<uint32_t>(fileEnd - 8));
  }

  // Patch avih::dwTotalFrames.
  {
    const uint32_t n = le32(frameCount);
    std::fseek(fp, pos_avih_totalframes, SEEK_SET);
    std::fwrite(&n, sizeof(n), 1, fp);
  }

  // Patch avih::dwSuggestedBufferSize and dwMaxBytesPerSec.
  {
    const uint32_t bufSize = le32(maxFrameBytes);
    std::fseek(fp, pos_avih_suggestedbuf, SEEK_SET);
    std::fwrite(&bufSize, sizeof(bufSize), 1, fp);

    // dwMaxBytesPerSec: rough upper bound = largest frame * fps
    const uint32_t maxBps = le32(static_cast<uint32_t>(maxFrameBytes * (fps + 0.5)));
    std::fseek(fp, pos_avih_maxbytespersec, SEEK_SET);
    std::fwrite(&maxBps, sizeof(maxBps), 1, fp);
  }

  // Patch strh::dwLength and strh::dwSuggestedBufferSize.
  {
    const uint32_t n = le32(frameCount);
    std::fseek(fp, pos_strh_length, SEEK_SET);
    std::fwrite(&n, sizeof(n), 1, fp);

    const uint32_t bufSize = le32(maxFrameBytes);
    std::fseek(fp, pos_strh_suggestedbuf, SEEK_SET);
    std::fwrite(&bufSize, sizeof(bufSize), 1, fp);
  }

  std::fflush(fp);
  std::fclose(fp);
  fp = nullptr;
  return true;
}

// ── CVideoFileWriter public API ───────────────────────────────────────────

CVideoFileWriter::CVideoFileWriter() : m_impl(std::make_unique<Impl>()) {}

CVideoFileWriter::~CVideoFileWriter() = default;

CVideoFileWriter::CVideoFileWriter(CVideoFileWriter&&) noexcept = default;
CVideoFileWriter& CVideoFileWriter::operator=(CVideoFileWriter&&) noexcept = default;

bool CVideoFileWriter::open(
    const std::string& out_file,
    double fps,
    const TImageSize& frameSize,
    VideoCodec codec,
    int jpeg_quality)
{
  close();  // finalise any previously open file

  m_impl->codec = codec;
  m_impl->width = frameSize.x;
  m_impl->height = frameSize.y;
  m_impl->fps = fps;
  m_impl->jpegQuality = jpeg_quality;
  m_impl->frameCount = 0;
  m_impl->maxFrameBytes = 0;
  m_impl->isGrayscale = false;
  m_impl->headersWritten = false;
  m_impl->index.clear();

  m_impl->fp = std::fopen(out_file.c_str(), "wb");
  return (m_impl->fp != nullptr);
}

void CVideoFileWriter::close()
{
  if (m_impl->fp == nullptr)
  {
    return;
  }
  m_impl->finalizeFile();
}

bool CVideoFileWriter::isOpen() const noexcept { return m_impl->fp != nullptr; }

VideoCodec CVideoFileWriter::codec() const noexcept { return m_impl->codec; }

TImageSize CVideoFileWriter::frameSize() const noexcept { return {m_impl->width, m_impl->height}; }

uint32_t CVideoFileWriter::frameCount() const noexcept { return m_impl->frameCount; }

const CVideoFileWriter& CVideoFileWriter::operator<<(const mrpt::img::CImage& img)
{
  if (!writeImage(img))
  {
    throw std::runtime_error("[CVideoFileWriter] Failed to write video frame");
  }
  return *this;
}

bool CVideoFileWriter::writeImage(const mrpt::img::CImage& img)
{
  if (!isOpen())
  {
    return false;
  }
  return m_impl->appendFrame(img);
}

}  // namespace mrpt::img