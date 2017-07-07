/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
// $Id: octomap_types.h 436 2012-10-15 10:18:16Z ahornung $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_TYPES_H
#define OCTOMAP_TYPES_H

#include <vector>
#include <list>

#include <cstdint>

#include <mrpt/otherlibs/octomap/math/Vector3.h>
#include <mrpt/otherlibs/octomap/math/Pose6D.h>
#include <mrpt/otherlibs/octomap/octomap_deprecated.h>

namespace octomap {

  ///Use Vector3 (float precision) as a point3d in octomap
  typedef octomath::Vector3               point3d;
  /// Use our Pose6D (float precision) as pose6d in octomap
  typedef octomath::Pose6D                pose6d;

  typedef std::vector<octomath::Vector3>  point3d_collection;
  typedef std::list<octomath::Vector3>    point3d_list;

  /// A voxel defined by its center point3d and its side length
  typedef std::pair<point3d, double> OcTreeVolume;

}

  //Macros for compiling with an without ROS (for output logging)
  #ifdef OCTOMAP_ROS
    #include <ros/ros.h>

    #define OCTOMAP_DEBUG         ROS_DEBUG
    #define OCTOMAP_DEBUG_STR     ROS_DEBUG_STREAM
    #define OCTOMAP_WARNING       ROS_WARN
    #define OCTOMAP_WARNING_STR   ROS_WARN_STREAM
    #define OCTOMAP_ERROR         ROS_ERROR
    #define OCTOMAP_ERROR_STR     ROS_ERROR_STREAM

  #else
    // no debug output if not in debug mode:
    #ifdef NDEBUG
      #ifndef OCTOMAP_NODEBUGOUT
        #define OCTOMAP_NODEBUGOUT
      #endif
    #endif

    #ifdef OCTOMAP_NODEBUGOUT
      #define OCTOMAP_DEBUG(...)       (void)0
      #define OCTOMAP_DEBUG_STR(...)   (void)0
    #else
      #define OCTOMAP_DEBUG(...)        fprintf(stderr, __VA_ARGS__), fflush(stderr)
      #define OCTOMAP_DEBUG_STR(args)   std::cerr << args << std::endl
    #endif

    #define OCTOMAP_WARNING(...)      fprintf(stderr, "WARNING: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
    #define OCTOMAP_WARNING_STR(args) std::cerr << "WARNING: " << args << std::endl
    #define OCTOMAP_ERROR(...)        fprintf(stderr, "ERROR: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
    #define OCTOMAP_ERROR_STR(args)   std::cerr << "ERROR: " << args << std::endl
  #endif


#endif
