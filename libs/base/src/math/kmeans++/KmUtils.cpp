/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// See KmUtils.h
//
// Author: David Arthur (darthur@gmail.com), 2009

#include "KmUtils.h"
#include <iostream>
using namespace std;

int __KMeansAssertionFailure(const char *file, int line, const char *expression) {
  cout << "ASSERTION FAILURE, " << file << " line " << line << ":" << endl;
  cout << "  " << expression << endl;
  exit(-1);
}
