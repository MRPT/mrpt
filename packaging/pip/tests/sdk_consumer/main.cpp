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

// Minimal C++ program built against the C++ SDK inside the MRPT wheel.

#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/config.h>

#include <atomic>
#include <iostream>

#if MRPT_HAS_TBB
#include <tbb/parallel_for.h>
#endif

int main()
{
  const mrpt::poses::CPose3D p(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
  const auto p2 = p + p;

  mrpt::maps::CSimplePointsMap pts;
  pts.insertPoint(1.0f, 2.0f, 3.0f);

  std::cout << "p+p=" << p2.asString() << " points=" << pts.size() << "\n";
  if (std::abs(p2.x() - 2.0) > 1e-9 || pts.size() != 1)
  {
    return 1;
  }

#if MRPT_HAS_TBB
  std::atomic<int> count{0};
  tbb::parallel_for(0, 1000, [&](int) { count++; });
  std::cout << "tbb count=" << count << "\n";
  if (count != 1000)
  {
    return 1;
  }
#endif

#if MRPT_HAS_NANOGUI
  // Only links against the GUI (no display needed): no window is created.
  const auto spin = &mrpt::gui::CDisplayWindowGUI::spinOnce;
  std::cout << "nanogui: " << (spin != nullptr ? "yes" : "no") << "\n";
#else
  std::cerr << "MRPT built without nanogui\n";
  return 1;
#endif
  return 0;
}
