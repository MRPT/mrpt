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

#include <mrpt/comms/CSerialPort.h>

#include <iostream>
#include <memory>

using namespace mrpt::comms;
using namespace std;

int main()
{
  try
  {
    std::unique_ptr<CSerialPort> serPort;

    string serName;

    std::cout << "Serial port test application: Use it with a loopback serial "
                 "port (pins 2-3 connected)"
              << "\n";

    std::cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0): ";
    getline(cin, serName);

    std::cout << "\n";
    std::cout << "Opening serial port...";
    serPort = std::make_unique<CSerialPort>(serName);
    std::cout << "OK"
              << "\n";

    std::cout << "Setting timeouts...";
    serPort->setTimeouts(100, 1, 100, 1, 100);
    std::cout << "OK"
              << "\n";

    std::cout << "Setting baud rate...";
    serPort->setConfig(500000);
    std::cout << "OK"
              << "\n";

    for (int i = 0; i < 10; i++)
    {
      // Test write:
      std::cout << "Writing test data...";
      const char buf1[] = "Hello world!";
      size_t written = serPort->Write(buf1, sizeof(buf1));
      std::cout << written << " bytes written."
                << "\n";

      // Read:
      std::cout << "Reading data...";
      char buf2[100];
      size_t nRead = serPort->Read(buf2, sizeof(buf2));
      std::cout << nRead << " bytes read: '";

      buf2[nRead] = 0;
      std::cout << buf2 << "'"
                << "\n";
    }
  }
  catch (const std::exception& e)
  {
    cerr << e.what() << "\n";
    return -1;
  }

  return 0;
}
