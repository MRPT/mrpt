/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** FILE: mrpt/base.h
	USE: This file includes all the headers of the library "libmrpt-base". 
  */
#ifndef mrpt_base_H
#define mrpt_base_H

#ifndef MRPT_NO_WARN_BIG_HDR
#include <mrpt/utils/core_defs.h>
MRPT_WARNING("Including <mrpt/base.h> makes compilation much slower, consider including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/config.h>
#include <mrpt/version.h>

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


#endif

