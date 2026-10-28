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

#include "points_cloudini_io.h"

#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CPointsMap.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <future>
#include <thread>

#include "cloudini_codec.h"

using namespace mrpt::maps;

CPointsMap::TSerializationOptions& CPointsMap::serializationOptions()
{
  static TSerializationOptions opts = []()
  {
    TSerializationOptions o;
    o.use_cloudini = mrpt::get_env<bool>("MRPT_POINTS_CLOUDINI", o.use_cloudini);
    o.xyz_resolution =
        mrpt::get_env<float>("MRPT_POINTS_CLOUDINI_XYZ_RESOLUTION", o.xyz_resolution);
    o.cloudini_min_points =
        mrpt::get_env<std::size_t>("MRPT_POINTS_CLOUDINI_MIN_POINTS", o.cloudini_min_points);
    return o;
  }();
  return opts;
}

bool CPointsMap::hasCloudini() { return internal::cloudini_available(); }

namespace mrpt::maps::internal
{
namespace
{
// Points per Cloudini stream: columns are split into chunks, coded in parallel.
constexpr uint32_t kChunkPoints = 1U << 16;

// Below this number of points (all columns), threads cost more than they save:
constexpr std::size_t kParallelMinPoints = 200'000;

// Quantized values must fit in the 64-bit integers of the encoder:
constexpr double kMaxQuantizedSteps = 1125899906842624.0;  // 2^50

std::size_t maxThreads() { return std::max(1U, std::thread::hardware_concurrency()); }

mrpt::WorkerThreadsPool& threadPool()
{
  static mrpt::WorkerThreadsPool pool(
      maxThreads(), mrpt::WorkerThreadsPool::POLICY_FIFO, "pts_cloudini");
  return pool;
}

// Waits for all the given tasks on destruction, so none outlives the data it
// uses if an exception is thrown.
template <typename FUTURES>
class WaitAllOnExit
{
 public:
  explicit WaitAllOnExit(FUTURES& f) : m_futures(f) {}
  ~WaitAllOnExit()
  {
    for (auto& f : m_futures)
    {
      if (f.valid())
      {
        f.wait();
      }
    }
  }
  WaitAllOnExit(const WaitAllOnExit&) = delete;
  WaitAllOnExit& operator=(const WaitAllOnExit&) = delete;

 private:
  FUTURES& m_futures;
};

bool runInParallel(const std::vector<PointColumn>& columns)
{
  std::size_t total = 0;
  for (const auto& c : columns)
  {
    total += c.n;
  }
  return total >= kParallelMinPoints && maxThreads() > 1;
}

// Calls f(T* data) with the given data pointer, typed as the column values.
template <typename F>
void visitColumn(const PointColumn& c, void* data, F&& f)
{
  switch (c.type)
  {
    case PointColumn::Type::Float:
      f(static_cast<float*>(data));
      break;
    case PointColumn::Type::Double:
      f(static_cast<double*>(data));
      break;
    case PointColumn::Type::UInt8:
      f(static_cast<uint8_t*>(data));
      break;
    case PointColumn::Type::UInt16:
      f(static_cast<uint16_t*>(data));
      break;
    case PointColumn::Type::UInt32:
      f(static_cast<uint32_t*>(data));
      break;
    default:
      THROW_EXCEPTION("Unknown point column type");
  }
}

void* columnData(const PointColumn& c) { return c.allocate ? c.allocate() : c.data; }

// Infinite values, or those too large for the quantization step, cannot be
// quantized: such chunks are stored exactly. NaN values are supported.
template <typename T>
std::optional<float> usableResolution(const T* data, std::size_t n, std::optional<float> res)
{
  if (!res || !(*res > 0))
  {
    return std::nullopt;
  }
  const double maxAbs = kMaxQuantizedSteps * static_cast<double>(*res);
  for (std::size_t i = 0; i < n; i++)
  {
    const double v = static_cast<double>(data[i]);
    if (std::isinf(v) || std::abs(v) > maxAbs)
    {
      return std::nullopt;
    }
  }
  return res;
}

// A range of points of one column, coded as one Cloudini stream:
struct Chunk
{
  std::size_t column = 0;
  std::size_t first = 0;
  std::size_t count = 0;
};

std::vector<Chunk> makeChunks(const std::vector<PointColumn>& columns, std::size_t chunkPoints)
{
  std::vector<Chunk> chunks;
  for (std::size_t i = 0; i < columns.size(); i++)
  {
    for (std::size_t first = 0; first < columns[i].n; first += chunkPoints)
    {
      chunks.push_back({i, first, std::min(chunkPoints, columns[i].n - first)});
    }
  }
  return chunks;
}

std::vector<uint8_t> encodeChunk(const PointColumn& c, const Chunk& chunk)
{
  std::vector<uint8_t> blob;
  visitColumn(
      c, c.data,
      [&](auto* data)
      {
        using T = std::remove_pointer_t<decltype(data)>;
        const T* d = data + chunk.first;
        std::optional<float> res;
        if constexpr (std::is_floating_point_v<T>)
        {
          res = usableResolution(d, chunk.count, c.resolution);
        }
        cloudini_encode<T>(d, chunk.count, res, blob);
      });
  return blob;
}

void decodeChunk(
    const std::vector<uint8_t>& blob, const PointColumn& c, void* data, const Chunk& chunk)
{
  visitColumn(
      c, data,
      [&](auto* d)
      {
        using T = std::remove_pointer_t<decltype(d)>;
        cloudini_decode<T>(blob.data(), blob.size(), d + chunk.first, chunk.count);
      });
}

void writeBlob(mrpt::serialization::CArchive& out, const std::vector<uint8_t>& blob)
{
  out.WriteAs<uint64_t>(blob.size());
  out.WriteBuffer(blob.data(), blob.size());
}

std::vector<uint8_t> readBlob(mrpt::serialization::CArchive& in)
{
  const auto size = in.ReadAs<uint64_t>();
  std::vector<uint8_t> blob(size);
  const auto nRead = in.ReadBuffer(blob.data(), blob.size());
  ASSERTMSG_(nRead == blob.size(), "Unexpected end of stream reading a Cloudini point field");
  return blob;
}

void writeCloudini(mrpt::serialization::CArchive& out, const std::vector<PointColumn>& columns)
{
  cloudini_require();
  const uint32_t chunkPoints = kChunkPoints;
  out.WriteAs<uint32_t>(chunkPoints);

  const auto chunks = makeChunks(columns, chunkPoints);
  if (!runInParallel(columns))
  {
    for (const auto& chunk : chunks)
    {
      writeBlob(out, encodeChunk(columns[chunk.column], chunk));
    }
    return;
  }
  // Encode chunks ahead in parallel, writing (and freeing) each one in order as
  // soon as it is ready:
  const std::size_t maxPending = 2 * maxThreads();
  std::deque<std::future<std::vector<uint8_t>>> pending;
  const WaitAllOnExit waitPending(pending);
  for (const auto& chunk : chunks)
  {
    if (pending.size() >= maxPending)
    {
      writeBlob(out, pending.front().get());
      pending.pop_front();
    }
    pending.push_back(threadPool().enqueue([&columns, &chunk]()
                                           { return encodeChunk(columns[chunk.column], chunk); }));
  }
  while (!pending.empty())
  {
    writeBlob(out, pending.front().get());
    pending.pop_front();
  }
}

void readCloudini(mrpt::serialization::CArchive& in, const std::vector<PointColumn>& columns)
{
  cloudini_require();

  const auto chunkPoints = in.ReadAs<uint32_t>();
  ASSERTMSG_(chunkPoints > 0, "Invalid Cloudini chunk size");

  const auto chunks = makeChunks(columns, chunkPoints);
  if (!runInParallel(columns))
  {
    std::vector<void*> data;
    for (const auto& c : columns)
    {
      data.push_back(columnData(c));
    }
    for (const auto& chunk : chunks)
    {
      decodeChunk(readBlob(in), columns[chunk.column], data[chunk.column], chunk);
    }
    return;
  }

  // Allocate the columns in parallel (zero-filling large arrays is slow) while
  // the first chunks are read. Being FIFO, the pool starts these tasks before
  // the decoding ones that wait for them.
  std::vector<std::shared_future<void*>> data;
  const WaitAllOnExit waitData(data);
  for (const auto& c : columns)
  {
    data.push_back(threadPool().enqueue([&c]() { return columnData(c); }).share());
  }
  // Decode each chunk while the next ones are read from the stream; each task
  // owns (and frees) its compressed data:
  const std::size_t maxPending = 2 * maxThreads();
  std::deque<std::future<void>> pending;
  const WaitAllOnExit waitPending(pending);
  for (const auto& chunk : chunks)
  {
    if (pending.size() >= maxPending)
    {
      pending.front().get();
      pending.pop_front();
    }
    pending.push_back(
        threadPool().enqueue([&columns, &chunk, colData = data[chunk.column], blob = readBlob(in)]()
                             { decodeChunk(blob, columns[chunk.column], colData.get(), chunk); }));
  }
  while (!pending.empty())
  {
    pending.front().get();
    pending.pop_front();
  }
  // Columns without points may not have been waited for:
  for (auto& d : data)
  {
    d.get();
  }
}

}  // namespace

uint8_t points_encoding_for_writing(std::size_t numPoints)
{
  const auto& o = CPointsMap::serializationOptions();
  return cloudini_available() && o.use_cloudini && numPoints >= o.cloudini_min_points
             ? kPointsEncodingCloudini
             : kPointsEncodingRaw;
}

float xyz_resolution() { return CPointsMap::serializationOptions().xyz_resolution; }

std::optional<float> field_resolution(const std::string& fieldName)
{
  const auto& r = CPointsMap::serializationOptions().field_resolutions;
  if (const auto it = r.find(fieldName); it != r.end())
  {
    return it->second;
  }
  return std::nullopt;
}

void write_point_columns(
    mrpt::serialization::CArchive& out, uint8_t encoding, const std::vector<PointColumn>& columns)
{
  switch (encoding)
  {
    case kPointsEncodingRaw:
      for (const auto& c : columns)
      {
        if (c.n == 0)
        {
          continue;
        }
        visitColumn(c, c.data, [&](const auto* data) { out.WriteBufferFixEndianness(data, c.n); });
      }
      break;
    case kPointsEncodingCloudini:
      writeCloudini(out, columns);
      break;
    default:
      THROW_EXCEPTION_FMT("Unknown point map encoding: %u", static_cast<unsigned>(encoding));
  }
}

void read_point_columns(
    mrpt::serialization::CArchive& in, uint8_t encoding, const std::vector<PointColumn>& columns)
{
  switch (encoding)
  {
    case kPointsEncodingRaw:
      for (const auto& c : columns)
      {
        void* data = columnData(c);
        if (c.n == 0)
        {
          continue;
        }
        visitColumn(c, data, [&](auto* d) { in.ReadBufferFixEndianness(d, c.n); });
      }
      break;
    case kPointsEncodingCloudini:
      readCloudini(in, columns);
      break;
    default:
      THROW_EXCEPTION_FMT(
          "Unknown point map encoding: %u (written by a newer version of MRPT?)",
          static_cast<unsigned>(encoding));
  }
}

}  // namespace mrpt::maps::internal
