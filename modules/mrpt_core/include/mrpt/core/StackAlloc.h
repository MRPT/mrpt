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
#include <new>  // std::bad_alloc

#if defined(_MSC_VER)
#include <malloc.h>
#elif defined(__GNUC__)
#include <alloca.h>
#endif

namespace mrpt
{

/// Maximum bytes we allow on the stack via alloca before falling back to heap.
/// Prevents silent stack overflows for large laser scans, images, etc.
inline constexpr std::size_t kStackAllocMaxBytes = 64U * 1024U;  // 64 KiB

/** Portable fast allocator: uses the stack for small buffers, heap otherwise.
 *
 * IMPORTANT: `alloca()` memory lives only in the *calling* function's stack
 * frame. Wrapping `alloca()` inside a helper/method means the memory is freed
 * when that helper returns — leaving a dangling pointer. Use the
 * MRPT_STACK_ALLOC() macro to ensure the alloca() call stays in the caller.
 *
 * \ingroup mrpt_core_grp
 */
template <typename T = std::byte>
class StackAlloc
{
 public:
  /// Construct from a pointer already obtained via alloca (or malloc).
  /// Prefer the MRPT_STACK_ALLOC() macro over calling this directly.
  StackAlloc(T* ptr, std::size_t count, bool on_heap) noexcept :
      data_(ptr), count_(count), owns_heap_(on_heap)
  {
  }

  /// Heap-only constructor (safe fallback, no macro needed).
  explicit StackAlloc(std::size_t count) : count_(count), owns_heap_(true)
  {
    data_ = static_cast<T*>(std::malloc(count * sizeof(T)));  // NOLINT
    if (data_ == nullptr)
    {
      throw std::bad_alloc();
    }
  }

  ~StackAlloc()
  {
    if (owns_heap_)
    {
      std::free(data_);  // NOLINT(cppcoreguidelines-no-malloc)
    }
  }

  // Non-copyable, non-movable:
  StackAlloc(const StackAlloc&) = delete;
  StackAlloc& operator=(const StackAlloc&) = delete;
  StackAlloc(StackAlloc&&) = delete;
  StackAlloc& operator=(StackAlloc&&) = delete;

  [[nodiscard]] T* get() noexcept { return data_; }
  [[nodiscard]] const T* get() const noexcept { return data_; }
  [[nodiscard]] std::size_t size() const noexcept { return count_; }

 private:
  T* data_ = nullptr;
  std::size_t count_ = 0;
  bool owns_heap_ = false;
};

}  // namespace mrpt

// ---------------------------------------------------------------------------
// Macro that keeps the alloca() call in the **caller's** stack frame.
// Falls back to heap when:
//   - the requested size exceeds kStackAllocMaxBytes, or
//   - the platform has no alloca support.
//
// Usage:
//   MRPT_STACK_ALLOC(float, myBuf, numElements);
//   float* ptr = myBuf.get();
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
#define MRPT_DETAIL_ALLOCA_(bytes) _alloca(bytes)
#define MRPT_DETAIL_HAS_ALLOCA_    1
#elif defined(__GNUC__)
#define MRPT_DETAIL_ALLOCA_(bytes) alloca(bytes) /* NOLINT */
#define MRPT_DETAIL_HAS_ALLOCA_    1             /* NOLINT */
#else
#define MRPT_DETAIL_ALLOCA_(bytes) nullptr
#define MRPT_DETAIL_HAS_ALLOCA_    0
#endif

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MRPT_STACK_ALLOC(Type, varName, count)                                          \
  const std::size_t varName##_nbytes_ = static_cast<std::size_t>(count) * sizeof(Type); \
  const bool varName##_use_heap_ =                                                      \
      (!MRPT_DETAIL_HAS_ALLOCA_) || (varName##_nbytes_ > mrpt::kStackAllocMaxBytes);    \
  mrpt::StackAlloc<Type> varName(                                                       \
      varName##_use_heap_                                                               \
          ? static_cast<Type*>(std::malloc(varName##_nbytes_))          /* NOLINT */    \
          : static_cast<Type*>(MRPT_DETAIL_ALLOCA_(varName##_nbytes_)), /* NOLINT */    \
      static_cast<std::size_t>(count), varName##_use_heap_)