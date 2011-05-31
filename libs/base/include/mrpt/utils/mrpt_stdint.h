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

#ifndef mrpt_stdint_H
#define mrpt_stdint_H

#include <mrpt/config.h>

// Define macros in platform dependant stdint.h header:
#ifndef __STDC_FORMAT_MACROS
#	define __STDC_FORMAT_MACROS
#endif
#ifndef __STDC_CONSTANT_MACROS
#	define __STDC_CONSTANT_MACROS
#endif
#ifndef __STDC_LIMIT_MACROS
#	define __STDC_LIMIT_MACROS
#endif

// Standard elemental types:
#if HAVE_STDINT_H
#	include <stdint.h>
#else
#	include "pstdint.h"  // The "portable stdint header file"
#endif



#endif

