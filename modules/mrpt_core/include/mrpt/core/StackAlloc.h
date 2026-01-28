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

#include <cstddef>
#include <cstdlib>

#if defined(_MSC_VER)
#include <malloc.h>
#elif defined(__GNUC__)
#include <alloca.h>
#endif

namespace mrpt
{
/** Portable stack allocator.
 *  Allocates memory on the stack when possible (MSVC, GCC), otherwise falls
 *  back to heap allocation.
 *
 * \ingroup mrpt_core_grp
 */
template <typename T = std::byte>
class StackAlloc
{
 public:
  explicit StackAlloc(std::size_t count)
  {
    if constexpr (supports_alloca())
    {
      data_ = static_cast<T*>(do_alloca(count * sizeof(T)));
      owns_heap_ = false;
    }
    else
    {
      data_ =
          static_cast<T*>(std::malloc(count * sizeof(T)));  // NOLINT(cppcoreguidelines-no-malloc)
      owns_heap_ = true;
    }
  }

  ~StackAlloc()
  {
    if (owns_heap_)
    {
      std::free(data_);  // NOLINT(cppcoreguidelines-no-malloc)
    }
  }

  // deleted copy constructor and assignment
  StackAlloc(const StackAlloc&) = delete;
  StackAlloc& operator=(const StackAlloc&) = delete;
  StackAlloc(StackAlloc&& other) = delete;
  StackAlloc& operator=(StackAlloc&& other) = delete;

  T* get() noexcept { return data_; }
  const T* get() const noexcept { return data_; }

 private:
  static constexpr bool supports_alloca() noexcept
  {
#if defined(_MSC_VER) || defined(__GNUC__)
    return true;
#else
    return false;
#endif
  }

  static void* do_alloca(std::size_t bytes)
  {
#if defined(_MSC_VER)
    return _alloca(bytes);
#elif defined(__GNUC__)
    return alloca(bytes);
#else
    return std::malloc(bytes);  // fallback
#endif
  }

  T* data_ = nullptr;
  bool owns_heap_ = false;
};

}  // namespace mrpt