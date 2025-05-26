/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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
  if (image_data)
  {
    stbi_image_free(image_data);
  }
  *this = {};
}

CImage::Impl::~Impl()
{
  stbi_image_free(image_data);
  image_data = nullptr;

  const thread_local bool SHOW_DEBUG_MSG = mrpt::get_env<bool>("MRPT_DEBUG_IMG_LAZY_LOAD", false);
  if (SHOW_DEBUG_MSG)
  {
    std::cout << "[CImage::dtor] Called on this=" << reinterpret_cast<const void*>(this) << "\n";
  }
}

}  // namespace mrpt::img
