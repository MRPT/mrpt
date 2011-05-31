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

/** FILE: mrpt/base.h
	USE: This file includes all the headers of the library "libmrpt-base".
  */
#ifndef mrpt_base_H
#define mrpt_base_H

#include <mrpt/config.h>
#include <mrpt/version.h>

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_base_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_base_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

#include <mrpt/compress.h>
#include <mrpt/math.h>
#include <mrpt/poses.h>
#include <mrpt/random.h>
#include <mrpt/synch.h>
#include <mrpt/system.h>
#include <mrpt/utils.h>

// These few headers in the namespace mrpt::bayes are in mrpt-base lib for now:
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CProbabilityParticle.h>

#endif // end precomp.headers


#endif

