/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/aligned_allocator.h>

#include <cstdlib>  // free, realloc, C++17 aligned_alloc
#include <cstring>  // memset

void* mrpt::aligned_calloc(size_t bytes, size_t alignment)
{
  void* ptr = mrpt::aligned_malloc(bytes, alignment);
  if (ptr)
  {
    ::memset(ptr, 0, bytes);
  }
  return ptr;
}
void* mrpt::aligned_malloc(size_t size, size_t alignment)
{
  // size must be an integral multiple of alignment:
  if (alignment != 0 && (size % alignment) != 0)
  {
    size = ((size / alignment) + 1) * alignment;
  }
#if defined(_MSC_VER) || defined(__MINGW32_MAJOR_VERSION)
  return _aligned_malloc(size, alignment);
#elif __APPLE__
  void* p;
  if (::posix_memalign(&p, alignment, size) != 0)
  {
    p = 0;
  }
  return p;
#else
  return ::aligned_alloc(alignment, size);
#endif
}
void mrpt::aligned_free(void* ptr)
{
#if defined(_MSC_VER) || defined(__MINGW32_MAJOR_VERSION)
  return _aligned_free(ptr);
#else
  return ::free(ptr);
#endif
}
