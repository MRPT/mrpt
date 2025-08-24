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
