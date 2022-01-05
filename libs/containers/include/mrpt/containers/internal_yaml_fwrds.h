/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <any>

// forward declarations
// clang-format off
// For auxiliary proxies:
namespace mrpt::containers { class yaml;
namespace internal {
 enum tag_as_proxy_t {}; enum tag_as_const_proxy_t {};
 template <typename T> T implAsGetter(const yaml& p);
 template <typename T> T implAnyAsGetter(const std::any& p);
}
// clang-format on
}  // namespace mrpt::containers
