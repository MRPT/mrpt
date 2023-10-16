#pragma once

#include <iostream>
#include <stdio.h>
#include <cstring>
#include <typeinfo>
#include <exception>
#include <type_traits>
#include <fstream>
#include "bonxai/bonxai.hpp"

#ifdef __GNUG__
#include <cstdlib>
#include <memory>
#include <cxxabi.h>
#endif

namespace Bonxai
{
/**
 * Serialize a grid to ostream. Easy :)
 */
template <typename DataT>
inline void Serialize(std::ostream& out, const VoxelGrid<DataT>& grid);

struct HeaderInfo
{
  std::string type_name;
  int inner_bits = 0;
  int leaf_bits = 0;
  double resolution = 0;
};

/**
 * @brief GetHeaderInfo is used to recover informations from the header of a
 * file/stream
 *
 * @param header  first line of the file
 */
inline HeaderInfo GetHeaderInfo(std::string header);

/**
 * @brief Deserialize create a grid. Note that template arguments need to be
 * consistent with HeaderInfo
 */
template <typename DataT>
inline VoxelGrid<DataT> Deserialize(std::istream& input, HeaderInfo info);

//---------------------------------------------------------
namespace details
{
#ifdef __GNUG__
inline std::string demangle(const char* name)
{
  int status = -4;  // some arbitrary value to eliminate the compiler warning

  std::unique_ptr<char, void (*)(void*)> res{
    abi::__cxa_demangle(name, NULL, NULL, &status), std::free
  };
  return (status == 0) ? res.get() : name;
}

#else

// does nothing if not g++
inline std::string demangle(const char* name)
{
  return name;
}
#endif

}  // namespace details

template <typename T>
inline void Write(std::ostream& out, const T& val)
{
  static_assert(std::is_trivially_copyable_v<T>, "Must be trivially copyable");
  out.write(reinterpret_cast<const char*>(&val), sizeof(T));
}

template <typename DataT>
inline void Serialize(std::ostream& out, const VoxelGrid<DataT>& grid)
{
  static_assert(std::is_trivially_copyable_v<DataT>,
                "DataT must ne trivially copyable");

  char header[256];
  std::string type_name = details::demangle(typeid(DataT).name());

  sprintf(header,
          "Bonxai::VoxelGrid<%s,%d,%d>(%lf)\n",
          type_name.c_str(),
          grid.INNER_BITS,
          grid.LEAF_BITS,
          grid.resolution);

  out.write(header, std::strlen(header));

  //------------
  Write(out, uint32_t(grid.root_map.size()));

  for (const auto& it : grid.root_map)
  {
    const CoordT& root_coord = it.first;
    Write(out, root_coord.x);
    Write(out, root_coord.y);
    Write(out, root_coord.z);

    const auto& inner_grid = it.second;
    for (size_t w = 0; w < inner_grid.mask().wordCount(); w++)
    {
      Write(out, inner_grid.mask().getWord(w));
    }
    for (auto inner = inner_grid.mask().beginOn(); inner; ++inner)
    {
      const uint32_t inner_index = *inner;
      const auto& leaf_grid = *(inner_grid.cell(inner_index));

      for (size_t w = 0; w < leaf_grid.mask().wordCount(); w++)
      {
        Write(out, leaf_grid.mask().getWord(w));
      }
      for (auto leaf = leaf_grid.mask().beginOn(); leaf; ++leaf)
      {
        const uint32_t leaf_index = *leaf;
        Write(out, leaf_grid.cell(leaf_index));
      }
    }
  }
}

template <typename T>
inline T Read(std::istream& input)
{
  T out;
  static_assert(std::is_trivially_copyable_v<T>, "Must be trivially copyable");
  input.read(reinterpret_cast<char*>(&out), sizeof(T));
  return out;
}

inline HeaderInfo GetHeaderInfo(std::string header)
{
  const std::string expected_prefix = "Bonxai::VoxelGrid<";
  if (header.rfind(expected_prefix, 0) != 0)
  {
    throw std::runtime_error("Header wasn't recognized");
  }
  int p1 = header.find(",", 18) + 1;
  auto part_type = header.substr(18, p1 - 18 - 1);

  int p2 = header.find(",", p1 + 1) + 1;
  auto part_ibits = header.substr(p1, p2 - p1 - 1);

  int p3 = header.find(">", p2) + 1;
  auto part_lbits = header.substr(p2, p3 - p2 - 1);

  int p4 = header.find("(", p3) + 1;
  int p5 = header.find(")", p4);
  auto part_res = header.substr(p4, p5 - p4);

  HeaderInfo info;
  info.type_name = part_type;
  info.inner_bits = std::stoi(part_ibits);
  info.leaf_bits = std::stoi(part_lbits);
  info.resolution = std::stod(part_res);

  return info;
}

template <typename DataT>
inline VoxelGrid<DataT> Deserialize(std::istream& input, HeaderInfo info)
{
  std::string type_name = details::demangle(typeid(DataT).name());
  if (type_name != info.type_name)
  {
    throw std::runtime_error("DataT does not match");
  }

  //------------

  VoxelGrid<DataT> grid(info.resolution, info.inner_bits, info.leaf_bits);

  uint32_t root_count = Read<uint32_t>(input);

  for (size_t root_index = 0; root_index < root_count; root_index++)
  {
    CoordT root_coord;
    root_coord.x = Read<int32_t>(input);
    root_coord.y = Read<int32_t>(input);
    root_coord.z = Read<int32_t>(input);

    auto inner_it = grid.root_map.find(root_coord);
    if (inner_it == grid.root_map.end())
    {
      inner_it =
          grid.root_map
              .insert({ root_coord,
                        typename VoxelGrid<DataT>::InnerGrid(info.inner_bits) })
              .first;
    }
    auto& inner_grid = inner_it->second;

    for (size_t w = 0; w < inner_grid.mask().wordCount(); w++)
    {
      uint64_t word = Read<uint64_t>(input);
      inner_grid.mask().setWord(w, word);
    }
    for (auto inner = inner_grid.mask().beginOn(); inner; ++inner)
    {
      const uint32_t inner_index = *inner;
      using LeafGridT = typename VoxelGrid<DataT>::LeafGrid;
      inner_grid.data[inner_index] = std::make_shared<LeafGridT>(info.leaf_bits);
      auto& leaf_grid = inner_grid.data[inner_index];

      for (size_t w = 0; w < leaf_grid->mask.wordCount(); w++)
      {
        uint64_t word = Read<uint64_t>(input);
        leaf_grid->mask.setWord(w, word);
      }
      for (auto leaf = leaf_grid->mask.beginOn(); leaf; ++leaf)
      {
        const uint32_t leaf_index = *leaf;
        leaf_grid->data[leaf_index] = Read<DataT>(input);
      }
    }
  }
  return grid;
}

}  // namespace Bonxai

