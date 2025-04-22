/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>

#include <cstdarg>  // For va_list, va_start, va_end, va_copy
#include <cstdio>   // For vsnprintf, _vscprintf (on MSVC)
#include <string>   // For std::string

#ifdef _MSC_VER
// Required on MSVC for _vscprintf and vsnprintf_s
#include <stdio.h>
#endif

namespace mrpt
{

// Note: function rewritten in 2025-05-20 using Google Gemini

// A sprintf-like function for std::string
std::string format(const char* fmt, ...)
{
  if (!fmt)
  {
    return {};  // Return empty string if format string is null
  }

  // --- Step 1: Determine required buffer size ---

  va_list args_for_size;
  va_start(args_for_size, fmt);

  // Use va_copy to duplicate the va_list for the size calculation,
  // as the size-calculation function might modify its state.
  va_list args_copy;
  va_copy(args_copy, args_for_size);

  int needed_size = -1;

#ifdef _MSC_VER
  // Microsoft-specific way to get the required buffer size
  needed_size = _vscprintf(fmt, args_copy);
  // _vscprintf returns -1 on error
#else
  // Standard C++11 / POSIX way to get the required buffer size
  needed_size = std::vsnprintf(nullptr, 0, fmt, args_copy);
  // vsnprintf returns a negative value on encoding error
#endif

  va_end(args_copy);  // Clean up the copied list

  // Check if size calculation failed
  if (needed_size < 0)
  {
    va_end(args_for_size);  // Clean up the original list before returning/throwing
    return "[format error: size calculation failed]";
  }

  // --- Step 2: Allocate buffer and format ---

  // Allocate the string buffer with the exact required size.
  // std::string manages memory and null termination.
  std::string buffer(static_cast<size_t>(needed_size), '\0');

  // --- Step 3: Format into the correctly sized buffer ---

  // Now format using the *original* va_list (args_for_size)
  // Pass buffer size + 1 to vsnprintf functions to include space for null terminator.
  int result = -1;

#ifdef _MSC_VER
  // Use vsnprintf_s on MSVC. Pass the buffer size as the count parameter
  // to prevent truncation now that we have the correct size.
  result = vsnprintf_s(&buffer[0], buffer.size() + 1, buffer.size(), fmt, args_for_size);
  // vsnprintf_s returns -1 on error, or the number of chars written.
#else
  // Use standard vsnprintf
  result = std::vsnprintf(&buffer[0], buffer.size() + 1, fmt, args_for_size);
  // vsnprintf returns negative on error, or the number of chars written.
#endif

  va_end(args_for_size);  // Clean up the original va_list

  // --- Step 4: Final checks and return ---

  // Check for errors during the actual formatting step
  if (result < 0)
  {
    return "[format error: formatting failed]";
  }

  // Sanity Check & Resize (Optional but good practice):
  // Ensure the string's size matches the number of characters actually written.
  // This handles cases where result might somehow be less than needed_size.
  // It shouldn't happen if the size calculation was correct, but it's safe to include.
  if (static_cast<size_t>(result) != buffer.size())
  {
    // This might occur if vsnprintf_s behaves differently than expected,
    // or if vsnprintf reported fewer characters written than needed.
    // Resizing ensures the std::string object's size is accurate.
    buffer.resize(static_cast<size_t>(result));
  }

  // Return the fully formatted string
  return buffer;
}

}  // namespace mrpt