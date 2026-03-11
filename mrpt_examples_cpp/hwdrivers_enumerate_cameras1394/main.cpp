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

#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>

#include <iostream>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace std;

// #define DO_CAPTURE		1
#define DO_CAPTURE 0

// ------------------------------------------------------
//				TestEnumerate_1394
// ------------------------------------------------------
void TestEnumerate_1394()
{
  CImageGrabber_dc1394::TCameraInfoList lstCams;

  std::cout << "Enumerating cameras..."
            << "\n";

  CImageGrabber_dc1394::enumerateCameras(lstCams);

  std::cout << "Found " << lstCams.size() << " cameras."
            << "\n";

  for (CImageGrabber_dc1394::TCameraInfoList::const_iterator it = lstCams.begin();
       it != lstCams.end(); it++)
  {
    std::cout << "======= CAMERA ========="
              << "\n";
    std::cout << "   GUID : " << it->guid << "\n";
    std::cout << "   Unit : " << it->unit << "\n";
    std::cout << "  Vendor: " << it->vendor << "\n";
    std::cout << "  Model : " << it->model << "\n";
    std::cout << "\n";
  }
}

int main(int argc, char** argv)
{
  try
  {
    TestEnumerate_1394();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}
