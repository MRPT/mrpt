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

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>

#include <iostream>

// ------------------------------------------------------
//				TestSE3
// ------------------------------------------------------
void TestSE3()
{
  using namespace mrpt;
  using namespace mrpt::poses;
  using namespace std;

  const CPose3D p0;
  const CPose3D p1(1, 2, 3, 0.0_deg, 0.0_deg, 0.0_deg);
  const CPose3D p2(1, 2, 3, 20.0_deg, 0.0_deg, 0.0_deg);

  cout << "p0: " << p0 << " SE(3)::log => " << Lie::SE<3>::log(p0) << endl;
  cout << "p1: " << p1 << " SE(3)::log => " << Lie::SE<3>::log(p1) << endl;
  cout << "p2: " << p2 << " SE(3)::log => " << Lie::SE<3>::log(p2) << endl;
}

int main()
{
  try
  {
    TestSE3();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception:" << mrpt::exception_to_str(e) << std::endl;
    return -1;
  }
}
