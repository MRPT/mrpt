/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef UTILSDEFS_H
#define UTILSDEFS_H

// ====== This header will be included in ALL mrpt libs and programs ========

#include <mrpt/config.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/boost_join.h>
#include <mrpt/utils/compiler_fixes.h>
#include <mrpt/utils/mrpt_macros.h>

// Linking pragmas for Win32
#include <mrpt/base/link_pragmas.h>

// Standard headers:
#include <cstddef>
#include <stdlib.h>
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

