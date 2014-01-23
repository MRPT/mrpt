/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

