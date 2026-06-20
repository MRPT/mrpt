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

#include "CImage_impl.h"

#include <mrpt/core/get_env.h>
#include <mrpt/img/CImage.h>

#include <iostream>

// Define STB_IMAGE_IMPLEMENTATION in exactly one .c or .cpp file
// before including the header to create the implementation.
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

STB_DISABLE_WARNINGS
#include "stb/stb_image.h"
#include "stb/stb_image_resize2.h"
#include "stb/stb_image_write.h"
STB_RESTORE_WARNINGS

namespace mrpt::img
{

void CImage::Impl::clear()
{
  if (image_data != nullptr)
  {
    stbi_image_free(image_data);
  }
  *this = {};
}

void CImage::Impl::clear_image_data()
{
  if (image_data == nullptr)
  {
    return;
  }
  stbi_image_free(image_data);
  image_data = nullptr;
  width = 0;
  height = 0;
}

CImage::Impl::~Impl()
{
  stbi_image_free(image_data);
  image_data = nullptr;

  const thread_local bool SHOW_DEBUG_MSG = mrpt::get_env<bool>("MRPT_DEBUG_IMG_LAZY_LOAD", false);
  if (SHOW_DEBUG_MSG)
  {
    std::cout << "[CImage::dtor] Called on this=" << reinterpret_cast<const void*>(this)  // NOLINT
              << "\n";
  }
}

}  // namespace mrpt::img
