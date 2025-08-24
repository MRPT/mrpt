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
// See KmUtils.h
//
// Author: David Arthur (darthur@gmail.com), 2009

#include "KmUtils.h"

#include <iostream>
using namespace std;

int __KMeansAssertionFailure(const char* file, int line, const char* expression)
{
  cout << "ASSERTION FAILURE, " << file << " line " << line << ":" << endl;
  cout << "  " << expression << endl;
  exit(-1);
}
