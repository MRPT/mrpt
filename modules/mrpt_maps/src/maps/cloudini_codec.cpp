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

// The only translation unit calling the Cloudini API. With Cloudini, it is built
// as C++20 (required by its headers); without it, as a stub that throws.

#include "cloudini_codec.h"

#include <stdexcept>

#if MRPT_MAPS_HAS_CLOUDINI
#include <cloudini_lib/cloudini.hpp>
#endif

namespace mrpt::maps::internal
{
#if MRPT_MAPS_HAS_CLOUDINI

namespace
{
template <typename T>
constexpr Cloudini::FieldType fieldTypeOf();
template <>
constexpr Cloudini::FieldType fieldTypeOf<float>()
{
  return Cloudini::FieldType::FLOAT32;
}
template <>
constexpr Cloudini::FieldType fieldTypeOf<double>()
{
  return Cloudini::FieldType::FLOAT64;
}
template <>
constexpr Cloudini::FieldType fieldTypeOf<uint8_t>()
{
  return Cloudini::FieldType::UINT8;
}
template <>
constexpr Cloudini::FieldType fieldTypeOf<uint16_t>()
{
  return Cloudini::FieldType::UINT16;
}
template <>
constexpr Cloudini::FieldType fieldTypeOf<uint32_t>()
{
  return Cloudini::FieldType::UINT32;
}
}  // namespace

bool cloudini_available() { return true; }

void cloudini_require() {}

template <typename T>
void cloudini_encode(
    const T* data, std::size_t n, std::optional<float> resolution, std::vector<uint8_t>& out)
{
  // Each field is encoded as a single-field cloud: MRPT stores fields as separate
  // contiguous arrays, which Cloudini reads in place, without interleaving copies.
  Cloudini::EncodingInfo info;
  info.fields.push_back({"v", 0, fieldTypeOf<T>(), resolution});
  info.width = static_cast<uint32_t>(n);
  info.height = 1;
  info.point_step = sizeof(T);
  // Only fields with a resolution are quantized; the rest are stored exactly:
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  // Callers already encode several fields in parallel:
  info.use_threads = false;

  Cloudini::PointcloudEncoder encoder(info);
  encoder.encode(Cloudini::ConstBufferView(data, n * sizeof(T)), out);
}

template <typename T>
void cloudini_decode(const uint8_t* stream, std::size_t streamSize, T* data, std::size_t n)
{
  Cloudini::ConstBufferView input(stream, streamSize);
  const auto info = Cloudini::DecodeHeader(input);
  if (info.fields.size() != 1 || info.fields[0].type != fieldTypeOf<T>() ||
      info.point_step != sizeof(T) || static_cast<std::size_t>(info.width) * info.height != n)
  {
    throw std::runtime_error(
        "Cloudini stream does not match the expected point field (type or point count)");
  }
  Cloudini::PointcloudDecoder decoder;
  Cloudini::BufferView output(data, n * sizeof(T));
  decoder.decode(info, input, output);
}

#else

namespace
{
[[noreturn]] void throwNoCloudini()
{
  throw std::runtime_error(
      "This point map was serialized with Cloudini compression, but this build of MRPT does "
      "not support Cloudini (rebuild mrpt_maps with the cloudini_lib dependency or the "
      "3rdparty/cloudini git submodule, on a little-endian platform)");
}
}  // namespace

bool cloudini_available() { return false; }

void cloudini_require() { throwNoCloudini(); }

template <typename T>
void cloudini_encode(
    [[maybe_unused]] const T* data,
    [[maybe_unused]] std::size_t n,
    [[maybe_unused]] std::optional<float> resolution,
    [[maybe_unused]] std::vector<uint8_t>& out)
{
  throwNoCloudini();
}

template <typename T>
void cloudini_decode(
    [[maybe_unused]] const uint8_t* stream,
    [[maybe_unused]] std::size_t streamSize,
    [[maybe_unused]] T* data,
    [[maybe_unused]] std::size_t n)
{
  throwNoCloudini();
}

#endif

// Explicit instantiations, for the point field types of mrpt::maps::CPointsMap:
template void cloudini_encode<float>(
    const float*, std::size_t, std::optional<float>, std::vector<uint8_t>&);
template void cloudini_decode<float>(const uint8_t*, std::size_t, float*, std::size_t);
template void cloudini_encode<double>(
    const double*, std::size_t, std::optional<float>, std::vector<uint8_t>&);
template void cloudini_decode<double>(const uint8_t*, std::size_t, double*, std::size_t);
template void cloudini_encode<uint8_t>(
    const uint8_t*, std::size_t, std::optional<float>, std::vector<uint8_t>&);
template void cloudini_decode<uint8_t>(const uint8_t*, std::size_t, uint8_t*, std::size_t);
template void cloudini_encode<uint16_t>(
    const uint16_t*, std::size_t, std::optional<float>, std::vector<uint8_t>&);
template void cloudini_decode<uint16_t>(const uint8_t*, std::size_t, uint16_t*, std::size_t);
template void cloudini_encode<uint32_t>(
    const uint32_t*, std::size_t, std::optional<float>, std::vector<uint8_t>&);
template void cloudini_decode<uint32_t>(const uint8_t*, std::size_t, uint32_t*, std::size_t);

}  // namespace mrpt::maps::internal
