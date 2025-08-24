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
/** \example serialization_stl/test.cpp */

#include <iostream>  // cout

template <class CONTAINER>
void printMap(const CONTAINER& m)
{
  for (const auto& e : m) std::cout << e.first << "=" << e.second << ", ";
  std::cout << std::endl;
}

//! [example]
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

#include <iostream>  // cout

void WriteAndReadExample()
{
  // Declare data to be serialized:
  std::map<std::string, uint32_t> m1{
      {"one", 1},
      {"two", 2}
  };

  // === Write ===
  {
    // CStream output:
    mrpt::io::CFileOutputStream ofs("file.bin");
    auto arch_out = mrpt::serialization::archiveFrom(ofs);
    // Use << to serialize in binary form:
    arch_out << m1;
  }

  // === Read ===
  std::map<std::string, uint32_t> m2;
  {
    // CStream output:
    mrpt::io::CFileInputStream ifs("file.bin");
    auto arch_in = mrpt::serialization::archiveFrom(ifs);
    // Use >> to deserialize:
    arch_in >> m2;
  }

  std::cout << "Wrote: ";
  printMap(m1);
  std::cout << "Read : ";
  printMap(m2);
}
//! [example]

//! [example_stdio]
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/archiveFrom_std_streams.h>
#include <mrpt/serialization/stl_serialization.h>

#include <fstream>   // io std streams
#include <iostream>  // cout

void WriteAndReadExampleStdIO()
{
  // Declare data to be serialized:
  std::map<std::string, uint32_t> m1{
      {"one", 1},
      {"two", 2}
  };

  // === Write ===
  {
    // CStream output:
    std::ofstream ofs("file.bin");
    auto arch_out = mrpt::serialization::archiveFrom<std::ostream>(ofs);
    // Use << to serialize in binary form:
    arch_out << m1;
  }

  // === Read ===
  std::map<std::string, uint32_t> m2;
  {
    // CStream output:
    std::ifstream ifs("file.bin");
    auto arch_in = mrpt::serialization::archiveFrom<std::istream>(ifs);
    // Use >> to deserialize:
    arch_in >> m2;
  }

  std::cout << "Wrote: ";
  printMap(m1);
  std::cout << "Read : ";
  printMap(m2);
}
//! [example_stdio]

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
  try
  {
    WriteAndReadExample();
    WriteAndReadExampleStdIO();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
    return -1;
  }
  catch (...)
  {
    printf("Untyped exception!");
    return -1;
  }
}
