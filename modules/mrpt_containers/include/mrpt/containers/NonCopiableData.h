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

#include <utility>  // std::move()

namespace mrpt::containers
{
/** A wrapper for a piece of data of type `T` which should not be copied
 *  or moved by default `operator =()`.
 *
 *  Useful for instance to hold a std::mutex or alike within a class or
 * structure with other regular data fields for which the default `operator =()`
 * is desired.
 *
 * \ingroup mrpt_containers_grp
 */
template <class T>
class NonCopiableData
{
 public:
  NonCopiableData() = default;
  ~NonCopiableData() = default;

  T data;

  NonCopiableData& operator=(const T& value)
  {
    data = value;
    return *this;
  }

  NonCopiableData(T&& value) : data(std::move(value)) {}
  NonCopiableData(const T& value) : data(value) {}

  T& operator*() { return data; }
  const T& operator*() const { return data; }
  T* operator->() { return &data; }
  const T* operator->() const { return &data; }

  NonCopiableData(const NonCopiableData&) {}
  NonCopiableData& operator=(const NonCopiableData&) { return *this; }

  NonCopiableData(NonCopiableData&&) {}
  NonCopiableData& operator=(NonCopiableData&&) { return *this; }
};

}  // namespace mrpt::containers
