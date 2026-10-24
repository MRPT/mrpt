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

// Serialization of point map fields (columns) compressed with Cloudini, shared
// by the CPointsMap classes.

#include <mrpt/serialization/CArchive.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace mrpt::maps::internal
{
/** Values of the "encoding" byte written before the point fields. */
constexpr uint8_t kPointsEncodingRaw = 0;
constexpr uint8_t kPointsEncodingCloudini = 1;

/** One point field: a contiguous array of `n` values. */
struct PointColumn
{
  enum class Type : uint8_t
  {
    Float,
    Double,
    UInt8,
    UInt16,
    UInt32
  };

  Type type = Type::Float;
  void* data = nullptr;
  std::size_t n = 0;
  /** Quantization step (float and double columns), std::nullopt: exact. */
  std::optional<float> resolution;
  /** If set, read_point_columns() calls it (maybe from another thread) to resize
   *  the destination array to `n` values and get its data pointer. */
  std::function<void*()> allocate;

  /** Column of the array `d`, which write_point_columns() only reads and
   *  read_point_columns() fills. `res` is ignored for integer types. */
  template <typename T>
  static PointColumn of(const T* d, std::size_t n, std::optional<float> res = std::nullopt)
  {
    PointColumn c;
    if constexpr (std::is_same_v<T, float>)
    {
      c.type = Type::Float;
    }
    else if constexpr (std::is_same_v<T, double>)
    {
      c.type = Type::Double;
    }
    else if constexpr (std::is_same_v<T, uint8_t>)
    {
      c.type = Type::UInt8;
    }
    else if constexpr (std::is_same_v<T, uint16_t>)
    {
      c.type = Type::UInt16;
    }
    else
    {
      static_assert(std::is_same_v<T, uint32_t>, "Unsupported point field type");
      c.type = Type::UInt32;
    }
    c.data = const_cast<T*>(d);
    c.n = n;
    if constexpr (std::is_floating_point_v<T>)
    {
      c.resolution = res;
    }
    return c;
  }

  /** Column to be read into the vector `v`, resized to `n` values right before. */
  template <typename VEC>
  static PointColumn resizing(VEC& v, std::size_t n)
  {
    PointColumn c = of(static_cast<const typename VEC::value_type*>(nullptr), n);
    c.allocate = [&v, n]() -> void*
    {
      v.resize(n);
      return v.data();
    };
    return c;
  }
};

/** The encoding to write a point map with: Cloudini if this build supports it
 *  and CPointsMap::serializationOptions() enable it for this number of points,
 *  raw otherwise. */
uint8_t points_encoding_for_writing(std::size_t numPoints);

/** Quantization step for the x,y,z columns. */
float xyz_resolution();

/** Quantization step for a float/double field, std::nullopt for an exact one. */
std::optional<float> field_resolution(const std::string& fieldName);

/** Writes the data of the columns with the given encoding. With Cloudini, the
 *  columns are split into chunks, encoded in parallel for large clouds. */
void write_point_columns(
    mrpt::serialization::CArchive& out, uint8_t encoding, const std::vector<PointColumn>& columns);

/** Reads what write_point_columns() wrote into the given columns: their arrays
 *  must hold `n` values, or be resized by PointColumn::allocate.
 *  \exception std::exception On an unknown encoding, or a Cloudini one if this
 *  build does not support it.
 */
void read_point_columns(
    mrpt::serialization::CArchive& in, uint8_t encoding, const std::vector<PointColumn>& columns);

}  // namespace mrpt::maps::internal
