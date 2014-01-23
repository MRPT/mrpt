/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef UTILSDEFS_H
#define UTILSDEFS_H

// ====== This header will be included in ALL mrpt libs and programs ========

#include <mrpt/config.h>
#include <mrpt/utils/compiler_fixes.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/boost_join.h>
#include <mrpt/utils/mrpt_macros.h>

// Linking pragmas for Win32
#include <mrpt/base/link_pragmas.h>

// Standard headers:
#include <cstddef>
//#include <stdlib.h>
#include <cstdlib>
#include <cmath>
#if HAVE_ALLOCA_H
# include <alloca.h>
#endif
// C++ STL Library:
#include <vector>
#include <set>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <limits>
#include <sstream>

// STL+ library:
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

// A few small functions and templates global to all mrpt libs:
#include <mrpt/utils/bits.h>

// Standard elemental types:
#include <mrpt/utils/types.h>

#endif

